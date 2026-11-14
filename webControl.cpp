#include "webControl.h"
#include "setup.h"
#include "HexCfg.h"
#include "webPage.h"
#include "calibration.h"
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include "serialOutput.h"

// HTTP runs on another task: one locked mailbox avoids concurrent gait writes or a stale movement
// queue.
static SemaphoreHandle_t lock;
static WebCommand pending;
static bool ready;
static uint32_t lastCommand;
static uint32_t owner;
static bool legacyOwner;
static char status[2048] = "{\"ready\":false}";

static esp_err_t jsonReply(httpd_req_t *req, const char *text)
{
  // A cached reply must not masquerade as current robot state or a newly accepted command.
  httpd_resp_set_type(req, "application/json");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  return httpd_resp_send(req, text, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t page(httpd_req_t *req)
{
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_set_hdr(req, "X-Frame-Options", "DENY");
  return httpd_resp_send(req, WEB_PAGE, sizeof(WEB_PAGE) - 1);
}

static esp_err_t state(httpd_req_t *req)
{
  char copy[sizeof(status)];
  xSemaphoreTake(lock, portMAX_DELAY);
  memcpy(copy, status, sizeof(copy));
  xSemaphoreGive(lock);
  return jsonReply(req, copy);
}

static esp_err_t command(httpd_req_t *req)
{
  char contentType[64];
  if (httpd_req_get_hdr_value_str(req, "Content-Type", contentType, sizeof(contentType)) !=
          ESP_OK ||
      strcmp(contentType, "application/json"))
  {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Use application/json");
  }
  if (httpd_req_get_hdr_value_len(req, "Origin"))
  {
    // Reject another website's browser requests; this local-network check is not user
    // authentication.
    char origin[128], host[96], expected[104];
    if (httpd_req_get_hdr_value_str(req, "Origin", origin, sizeof(origin)) != ESP_OK ||
        httpd_req_get_hdr_value_str(req, "Host", host, sizeof(host)) != ESP_OK)
    {
      return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid origin");
    }
    snprintf(expected, sizeof(expected), "http://%s", host);
    if (strcmp(origin, expected))
    {
      return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Cross-origin control denied");
    }
  }
  if (req->content_len == 0 || req->content_len >= 384)
  {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid command size");
  }
  char body[384];
  unsigned received = 0;
  // One HTTP body may arrive in fragments; validate only after reading the complete bounded
  // payload.
  while (received < req->content_len)
  {
    int count = httpd_req_recv(req, body + received, req->content_len - received);
    if (count <= 0)
    {
      return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Incomplete command");
    }
    received += count;
  }
  body[received] = 0;
  JsonDocument doc;
  if (deserializeJson(doc, body) || !doc["client"].is<uint32_t>() || !doc["enable"].is<bool>() ||
      !doc["stop"].is<bool>())
  {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid command");
  }
  WebCommand input = {};
  // Reject wrong types/ranges before narrowing axes or dispatching settings to the motion thread.
  const char *axes[] = {"lx", "ly", "rx", "ry"};
  int *values[] = {&input.lx, &input.ly, &input.rx, &input.ry};
  for (byte i = 0; i < 4; ++i)
  {
    if (!doc[axes[i]].is<int>() || doc[axes[i]].as<int>() < 0 || doc[axes[i]].as<int>() > 255)
    {
      return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid axis");
    }
    *values[i] = doc[axes[i]].as<int>();
  }
  const char *action = doc["action"].as<const char *>();
  if (!action || strlen(action) >= sizeof(input.action) || !doc["value"].is<int>())
  {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid action");
  }
  strcpy(input.action, action);
  input.value = doc["value"];
  int v = input.value;
  bool valid = (!action[0] && v == 0) || (!strcmp(action, "enable") && v == 1) ||
               ((!strcmp(action, "bodyUp") || !strcmp(action, "bodyDown") ||
                 !strcmp(action, "liftUp") || !strcmp(action, "liftDown")) &&
                v == 0) ||
               (!strcmp(action, "mode") && v >= 0 && v <= 3) ||
               (!strcmp(action, "gait") && v >= 0 && v <= 5) ||
               (!strcmp(action, "height") && (v == 45 || v == 56 || v == 65 || v == 85)) ||
               (!strcmp(action, "speed") && v >= 0 && v <= 100) ||
               (!strcmp(action, "lift") && v >= MIN_LEG_LIFT_MM && v <= MAX_LEG_LIFT_MM &&
                v % LEG_LIFT_STEP_MM == 0) ||
               ((!strcmp(action, "travel") || !strcmp(action, "balance") ||
                 !strcmp(action, "imu") || !strcmp(action, "feet") || !strcmp(action, "hold")) &&
                (v == 0 || v == 1)) ||
               (!strcmp(action, "leg") && ((v >= 0 && v <= 5) || v == 255));
  if (!valid)
  {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Unknown action or invalid value");
  }
  input.enable = doc["enable"];
  input.stop = doc["stop"];
  uint32_t client = doc["client"];
  if (!client)
  {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid client");
  }
  xSemaphoreTake(lock, portMAX_DELAY);
  if (calibration_busy())
  {
    xSemaphoreGive(lock);
    httpd_resp_set_status(req, "409 Conflict");
    return jsonReply(req, "{\"code\":\"calibration\",\"error\":\"Calibration owns motion\"}");
  }
  // Only one client/legacy transport owns movement; accepted commands renew a short watchdog lease.
  bool leased = uint32_t(millis() - lastCommand) < WEB_LEASE_MS;
  if ((legacyOwner || (owner && owner != client)) && leased)
  {
    xSemaphoreGive(lock);
    httpd_resp_set_status(req, "409 Conflict");
    return jsonReply(req, "{\"code\":\"owner\",\"error\":\"Another controller owns movement\"}");
  }
  if (ready && !input.stop && input.enable)
  {
    // Backpressure preserves the pending action; stop/disable may replace it to avoid waiting
    // behind motion.
    xSemaphoreGive(lock);
    httpd_resp_set_status(req, "409 Conflict");
    return jsonReply(req, "{\"code\":\"busy\",\"error\":\"Controller busy; retry latest input\"}");
  }
  input.ownershipChanged = legacyOwner || owner != client || !leased;
  // Takeover/expiry must be visible to the loop so another client's movement cannot inherit arming.
  legacyOwner = false;
  owner = client;
  lastCommand = millis();
  pending = input;
  ready = true;
  xSemaphoreGive(lock);
  return jsonReply(req, "{\"accepted\":true}");
}

void webControl_register(httpd_handle_t server)
{
  lock = xSemaphoreCreateMutex();
  if (!lock)
  {
    Serial.println("Web controller mutex allocation failed.");
    return;
  }
  const char *paths[] = {"/", "/api/status", "/api/control"};
  esp_err_t (*handlers[])(httpd_req_t *) = {page, state, command};
  for (byte i = 0; i < 3; ++i)
  {
    httpd_uri_t uri = {};
    uri.uri = paths[i];
    uri.method = i == 2 ? HTTP_POST : HTTP_GET;
    uri.handler = handlers[i];
    if (httpd_register_uri_handler(server, &uri) != ESP_OK)
    {
      Serial.printf("Cannot register web route %s\n", paths[i]);
    }
  }
}

bool webControl_take(WebCommand &command)
{
  if (!lock)
  {
    return false;
  }
  xSemaphoreTake(lock, portMAX_DELAY);
  bool available = ready && !legacyOwner && uint32_t(millis() - lastCommand) < WEB_LEASE_MS;
  // A blocked main loop must not execute an enable/movement request after its lease has already
  // expired.
  if (available)
  {
    command = pending;
    ready = false;
  }
  else if (ready)
  {
    ready = false;
    Serial.println("Expired web command discarded; enable explicitly.");
  }
  xSemaphoreGive(lock);
  return available;
}

bool webControl_expired()
{
  if (!lock)
  {
    return true;
  }
  xSemaphoreTake(lock, portMAX_DELAY);
  bool expired = legacyOwner || !owner || uint32_t(millis() - lastCommand) >= WEB_LEASE_MS;
  xSemaphoreGive(lock);
  return expired;
}

bool webControl_claimLegacy()
{
  // Optional old-app control participates in the same lease instead of racing the browser.
  if (!lock)
  {
    return false;
  }
  xSemaphoreTake(lock, portMAX_DELAY);
  bool allowed =
      (!owner || legacyOwner || uint32_t(millis() - lastCommand) >= WEB_LEASE_MS) && !ready &&
      !calibration_busy();
  if (allowed)
  {
    legacyOwner = true;
    owner = 0;
    lastCommand = millis();
  }
  xSemaphoreGive(lock);
  return allowed;
}

void webControl_status(const char *json)
{
  // Publish a serialized snapshot so HTTP can report telemetry without touching I2C or live gait
  // state.
  if (!lock)
  {
    return;
  }
  xSemaphoreTake(lock, portMAX_DELAY);
  snprintf(status, sizeof(status), "%s", json);
  xSemaphoreGive(lock);
}
