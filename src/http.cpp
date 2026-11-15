#include <ArduinoJson.h>
#include <WiFi.h>
#include "esp_http_server.h"
#include "camera.h"
#include "http.h"
#include "setup.h"
#include <HTTPClient.h>
#include "battMan.h"
#include "webControl.h"
#include "wifey.h"
#include "ota.h"
#include "calibration.h"
#include <math.h>
#include "serialOutput.h"

static bool httpDataReady = false;
static byte httpInputData[7];
// HTTP handlers and the Arduino loop run on different tasks; publish complete packets/snapshots
// atomically.
static portMUX_TYPE httpLock = portMUX_INITIALIZER_UNLOCKED;
static float cachedCell1, cachedCell2;
static bool voltageReady;
static uint32_t voltagePublished;

/**
 * @brief HTTP POST handler for the "/values" endpoint.
 *
 * Expects a JSON body containing keys value0..value6. Parsed bytes are stored
 * into the internal httpInputData buffer and the httpDataReady flag is set.
 *
 * @param req Pointer to the incoming HTTP request
 * @return ESP_OK on success, ESP_FAIL on parse/recv error
 */
static esp_err_t data_handler(httpd_req_t *req)
{
  // Read the data from the request
  const size_t bufferSize = 1024;
  char buf[bufferSize];
  size_t recvLen = req->content_len;

  if (recvLen == 0 || recvLen >= bufferSize)
  {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Request too large");
  }

  int ret = 0;
  // TCP may split one JSON command across reads; never parse a partial control packet.
  while (size_t(ret) < recvLen)
  {
    int count = httpd_req_recv(req, buf + ret, recvLen - ret);
    if (count <= 0)
    {
      return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Incomplete request");
    }
    ret += count;
  }
  if (ret <= 0)
  {
    if (ret == HTTPD_SOCK_ERR_TIMEOUT)
    {
      httpd_resp_send_408(req);
    }
    return ESP_FAIL;
  }

  buf[ret] = '\0';

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, buf);
  if (error)
  {
#ifdef DEBUG_HTTP_DATA
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
#endif // DEBUG_HTTP_DATA
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid JSON");
  }
  // Validate into a local packet so malformed input cannot partially overwrite the active command.
  byte command[7];
  for (int i = 0; i < 7; i++)
  {
    char key[8];
    snprintf(key, sizeof(key), "value%d", i);
    if (!doc[key].is<int>() || doc[key].as<int>() < 0 || doc[key].as<int>() > 255)
    {
      return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Expected seven bytes value0..value6");
    }
    command[i] = doc[key];
#ifdef DEBUG_HTTP_DATA
    Serial.print(httpInputData[i]);
#endif // DEBUG_HTTP_DATA
  }
  unsigned checksum = 0;
  for (int i = 1; i < 7; ++i)
  {
    checksum += command[i];
  }
  if (command[0] != byte((checksum >> 8) + (checksum & 255)))
  {
    return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Invalid checksum");
  }
  if (calibration_busy())
  {
    httpd_resp_set_status(req, "409 Conflict");
    return httpd_resp_send(req, "Calibration owns motion", HTTPD_RESP_USE_STRLEN);
  }
#ifdef WEB_CONTROLLER_ENABLED
  if (!webControl_claimLegacy())
  {
    httpd_resp_set_status(req, "409 Conflict");
    return httpd_resp_send(req, "Web controller owns input", HTTPD_RESP_USE_STRLEN);
  }
#endif
  portENTER_CRITICAL(&httpLock);
  memcpy(httpInputData, command, sizeof(command));
#ifdef DEBUG_HTTP_DATA
  Serial.println();
#endif // DEBUG_HTTP_DATA

  httpDataReady = true;
  portEXIT_CRITICAL(&httpLock);
  httpd_resp_send(req, "Commands received", HTTPD_RESP_USE_STRLEN);

  return ESP_OK;
}

/**
 * @brief HTTP GET handler for the "/battInfo" endpoint.
 *
 * Returns { "cell1": <float>, "cell2": <float> }. All builds use the main-loop
 * snapshot to avoid concurrent HTTP-task I2C access in all builds.
 *
 * @param req Pointer to the incoming HTTP request
 * @return ESP_OK
 */
static esp_err_t batt_info_handler(httpd_req_t *req)
{
  float v1 = 0.0f, v2 = 0.0f;
  portENTER_CRITICAL(&httpLock);
  bool available = voltageReady && uint32_t(millis() - voltagePublished) <= 500;
  v1 = cachedCell1;
  v2 = cachedCell2;
  portEXIT_CRITICAL(&httpLock);
  if (!available)
  {
    httpd_resp_set_status(req, "503 Service Unavailable");
    return httpd_resp_send(req, "Battery sample unavailable", HTTPD_RESP_USE_STRLEN);
  }

  JsonDocument doc;
  doc["cell1"] = v1;
  doc["cell2"] = v2;

  String out;
  serializeJson(doc, out);

  httpd_resp_set_type(req, "application/json");
  httpd_resp_send(req, out.c_str(), HTTPD_RESP_USE_STRLEN);
  return ESP_OK;
}

void http_setVoltages(float cell1, float cell2)
{
  portENTER_CRITICAL(&httpLock);
  cachedCell1 = cell1;
  cachedCell2 = cell2;
  voltageReady = isfinite(cell1) && isfinite(cell2);
  voltagePublished = millis();
  portEXIT_CRITICAL(&httpLock);
}

bool http_takeInput(byte *command)
{
  // Consume and clear under one lock so the next HTTP packet cannot be lost between separate calls.
  portENTER_CRITICAL(&httpLock);
  bool available = httpDataReady;
  if (available)
  {
    memcpy(command, httpInputData, sizeof(httpInputData));
    httpDataReady = false;
  }
  portEXIT_CRITICAL(&httpLock);
  return available;
}

void http_init()
{
  // Prevent the lwIP Invalid mbox crash when missing credentials made Wi-Fi initialization return
  // early.
  if (!wifi_ready())
  {
    Serial.println("HTTP servers not started: configure Wi-Fi over Serial, then reset.");
    return;
  }
  httpd_handle_t data_httpd = NULL;
  httpd_config_t data_config = HTTPD_DEFAULT_CONFIG();
  data_config.server_port = 80;
  data_config.max_uri_handlers = 12;

  httpd_handle_t stream_httpd = NULL;
  httpd_config_t stream_config = HTTPD_DEFAULT_CONFIG();
  stream_config.task_priority = tskIDLE_PRIORITY + 4;
  stream_config.server_port = 81;
  stream_config.ctrl_port = 30000;

  httpd_uri_t data_uri = {
      .uri = "/values", .method = HTTP_POST, .handler = data_handler, .user_ctx = NULL};

  httpd_uri_t stream_uri = {
      .uri = "/stream", .method = HTTP_GET, .handler = camera_stream_handler, .user_ctx = NULL};

  httpd_uri_t batt_uri = {
      .uri = "/battInfo", .method = HTTP_GET, .handler = batt_info_handler, .user_ctx = NULL};

  if (httpd_start(&data_httpd, &data_config) == ESP_OK)
  {
#if defined(LEGACY_HTTP_COMMAND_ENABLED) || !defined(WEB_CONTROLLER_ENABLED)
    // Keep old app compatibility opt-in instead of exposing a second default movement endpoint.
    httpd_register_uri_handler(data_httpd, &data_uri);
#endif
#ifdef WEB_CONTROLLER_ENABLED
    webControl_register(data_httpd);
#endif
    ota_register(data_httpd);
    calibration_register(data_httpd);
    httpd_register_uri_handler(data_httpd, &batt_uri);
  }
  else
  {
    Serial.println("HTTP control server failed to start.");
  }

#ifdef CAMERA_ENABLED
  if (httpd_start(&stream_httpd, &stream_config) == ESP_OK)
  {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
  else
  {
    Serial.println("Camera stream server failed to start.");
  }
#endif
}

byte *http_inputData()
{
  return httpInputData;
}

bool http_dataReady()
{
  return httpDataReady;
}

void http_setDataReady(bool dataStatus)
{
  httpDataReady = dataStatus;
}

void http_resetInputData()
{
  memset(httpInputData, 0, sizeof(httpInputData));
}