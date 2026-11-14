#include "ota.h"
#include <Arduino.h>
#include <Update.h>
#include <atomic>
#include "esp_ota_ops.h"
#include "serialOutput.h"

enum class OtaState { IDLE, REQUESTED, REJECTED, LOCKED };
static std::atomic<OtaState> state{OtaState::IDLE};
static std::atomic<bool> restartPending{false};

static const char UPDATE_PAGE[] = R"HTML(<!doctype html>
<html lang="en"><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>Hexapod firmware update</title>
<h1>Firmware update</h1>
<p>No authentication. Use a trusted private network only.</p>
<p>Support the robot securely. Disable Control and Camera and close other control tabs.
Updating restarts the robot and runs startup positioning. Do not disconnect power.</p>
<p>Select the application firmware.bin built for this robot, NOT a bootloader,
partition table or merged flash image. A failed transfer requires a reset.</p>
<input id="file" type="file" accept=".bin"><button id="upload">Upload firmware</button>
<p id="status" role="status">No upload started.</p><a href="/">Back to controller</a>
<script>
const file = document.getElementById("file"), button = document.getElementById("upload"),
  status = document.getElementById("status");
button.onclick = async () => {
  const image = file.files[0];
  if (!image || !image.name.toLowerCase().endsWith(".bin") || !image.size) {
    status.textContent = "Choose a nonempty application .bin file."; return;
  }
  if (!confirm("Robot supported, Control disabled, Camera off? Upload and restart?")) return;
  button.disabled = true;
  status.textContent = "Uploading and flashing. Keep power connected; wait for the result.";
  try {
    const response = await fetch("/update", {
      method: "POST", headers: {"Content-Type": "application/octet-stream"}, body: image
    });
    const text = await response.text();
    if (!response.ok) throw Error(text);
    status.textContent = text;
  } catch (error) {
    status.textContent = "Update failed: " + error.message +
      " If transfer started, reset before retrying. Do not assume firmware was installed.";
  } finally { button.disabled = false; }
};
</script></html>)HTML";

bool ota_locked()
{
  return state.load() == OtaState::LOCKED;
}

void ota_update(bool allowUpdate)
{
  OtaState expected = OtaState::REQUESTED;
  state.compare_exchange_strong(expected, allowUpdate ? OtaState::LOCKED : OtaState::REJECTED);
  if (restartPending.load())
  {
    delay(1000);
    ESP.restart();
  }
  if (ota_locked())
  {
    delay(1);
  }
}

static esp_err_t updatePage(httpd_req_t *req)
{
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  httpd_resp_set_hdr(req, "X-Frame-Options", "DENY");
  return httpd_resp_send(req, UPDATE_PAGE, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t updateError(httpd_req_t *req, const char *status, const char *message)
{
  Serial.println(message);
  httpd_resp_set_status(req, status);
  httpd_resp_set_hdr(req, "Connection", "close");
  httpd_resp_send(req, message, HTTPD_RESP_USE_STRLEN);
  return ESP_FAIL;
}

static esp_err_t uploadFirmware(httpd_req_t *req)
{
  char type[64], origin[128], host[96], expectedOrigin[104];
  httpd_resp_set_type(req, "text/plain");
  httpd_resp_set_hdr(req, "Cache-Control", "no-store");
  if (httpd_req_get_hdr_value_str(req, "Content-Type", type, sizeof(type)) != ESP_OK ||
      strcmp(type, "application/octet-stream"))
  {
    return updateError(req, "400 Bad Request", "Use the update page to upload an application .bin.");
  }
  if (httpd_req_get_hdr_value_len(req, "Origin"))
  {
    if (httpd_req_get_hdr_value_str(req, "Origin", origin, sizeof(origin)) != ESP_OK ||
        httpd_req_get_hdr_value_str(req, "Host", host, sizeof(host)) != ESP_OK)
    {
      return updateError(req, "403 Forbidden", "Invalid update origin.");
    }
    snprintf(expectedOrigin, sizeof(expectedOrigin), "http://%s", host);
    if (strcmp(origin, expectedOrigin))
    {
      return updateError(req, "403 Forbidden", "Cross-origin firmware updates are not allowed.");
    }
  }
  const esp_partition_t *partition = esp_ota_get_next_update_partition(nullptr);
  if (!partition || !req->content_len || req->content_len > partition->size)
  {
    return updateError(req, "400 Bad Request", "Empty/oversized image or no OTA application slot. Install OTA layout by USB.");
  }
  OtaState expected = OtaState::IDLE;
  if (!state.compare_exchange_strong(expected, OtaState::REQUESTED))
  {
    return updateError(req, "409 Conflict", "Update already requested or motion locked; reset before retrying.");
  }
  // Only the motion thread may acknowledge that both Control and motion are disabled.
  for (unsigned i = 0; i < 1000 && state.load() == OtaState::REQUESTED; ++i)
  {
    delay(1);
  }
  expected = OtaState::REQUESTED;
  if (state.compare_exchange_strong(expected, OtaState::IDLE))
  {
    return updateError(req, "503 Service Unavailable", "Motion thread did not acknowledge update; nothing flashed.");
  }
  if (state.load() == OtaState::REJECTED)
  {
    state.store(OtaState::IDLE);
    return updateError(req, "409 Conflict", "Disable Control and wait for motion to be disabled before updating.");
  }
  Serial.println("Browser firmware transfer started; motion locked until restart.");
  if (!Update.begin(req->content_len, U_FLASH))
  {
    return updateError(req, "500 Internal Server Error", "Cannot start flash update; reset before retrying.");
  }
  uint8_t buffer[1024];
  size_t remaining = req->content_len;
  while (remaining)
  {
    int count = httpd_req_recv(req, reinterpret_cast<char *>(buffer),
                              remaining < sizeof(buffer) ? remaining : sizeof(buffer));
    if (count <= 0 || Update.write(buffer, size_t(count)) != size_t(count))
    {
      Update.abort();
      return updateError(req, "500 Internal Server Error", "Incomplete upload or flash write failure; reset before retrying.");
    }
    remaining -= size_t(count);
  }
  if (!Update.end())
  {
    Update.abort();
    return updateError(req, "400 Bad Request", "Firmware validation failed; use the correct application image and reset before retrying.");
  }
  Serial.println("Browser update complete; restarting.");
  esp_err_t result = httpd_resp_send(req, "Firmware installed. Restarting; refresh the controller after startup.", HTTPD_RESP_USE_STRLEN);
  restartPending.store(true);
  return result;
}

void ota_register(httpd_handle_t server)
{
  const char *paths[] = {"/update", "/update"};
  esp_err_t (*handlers[])(httpd_req_t *) = {updatePage, uploadFirmware};
  for (unsigned i = 0; i < 2; ++i)
  {
    httpd_uri_t uri = {};
    uri.uri = paths[i];
    uri.method = i == 0 ? HTTP_GET : HTTP_POST;
    uri.handler = handlers[i];
    if (httpd_register_uri_handler(server, &uri) != ESP_OK)
    {
      Serial.println("Cannot register browser firmware update route.");
    }
  }
}
