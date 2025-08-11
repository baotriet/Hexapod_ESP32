#include <ArduinoJson.h>
#include <WiFi.h>
#include "esp_http_server.h"
#include "camera.h"
#include "http.h"
#include "setup.h"

JsonDocument doc;

static bool httpDataReady = false;
static byte httpInputData[7];

static esp_err_t data_handler(httpd_req_t *req) {
  // Read the data from the request
  const size_t bufferSize = 1024;
  char buf[bufferSize];
  size_t recvLen = req->content_len;

  if (recvLen > bufferSize) {
    recvLen = bufferSize;
  }

  int ret = httpd_req_recv(req, buf, recvLen);
  if (ret <= 0) {
    if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
      httpd_resp_send_408(req);
    }
    return ESP_FAIL;
  }

  buf[ret] = '\0';

  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, buf);
  if (error) {
#ifdef DEBUG_HTTP_DATA
    Serial.print(F("deserializeJson() failed: "));
    Serial.println(error.f_str());
#endif  // DEBUG_HTTP_DATA
    return ESP_FAIL;
  }
  for (int i = 0; i < 7; i++) {
    char key[8];
    snprintf(key, sizeof(key), "value%d", i);
    httpInputData[i] = doc[key];
#ifdef DEBUG_HTTP_DATA
    Serial.print(httpInputData[i]);
#endif  // DEBUG_HTTP_DATA
  }
#ifdef DEBUG_HTTP_DATA
  Serial.println();
#endif  // DEBUG_HTTP_DATA

  httpDataReady = true;
  httpd_resp_send(req, "Commands received", HTTPD_RESP_USE_STRLEN);

  return ESP_OK;
}

void http_init() {
  httpd_handle_t data_httpd = NULL;
  httpd_config_t data_config = HTTPD_DEFAULT_CONFIG();
  data_config.server_port = 80;

  httpd_handle_t stream_httpd = NULL;
  httpd_config_t stream_config = HTTPD_DEFAULT_CONFIG();
  stream_config.task_priority = tskIDLE_PRIORITY + 4;
  stream_config.server_port = 81;
  stream_config.ctrl_port = 30000;

  httpd_uri_t data_uri = {
    .uri = "/values",
    .method = HTTP_POST,
    .handler = data_handler,
    .user_ctx = NULL
  };

  httpd_uri_t stream_uri = {
    .uri = "/stream",
    .method = HTTP_GET,
    .handler = camera_stream_handler,
    .user_ctx = NULL
  };

  if (httpd_start(&data_httpd, &data_config) == ESP_OK) {
    httpd_register_uri_handler(data_httpd, &data_uri);
  }

  if (httpd_start(&stream_httpd, &stream_config) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

byte *http_inputData() {
  return httpInputData;
}

bool http_dataReady() {
  return httpDataReady;
}

void http_setDataReady(bool dataStatus) {
  httpDataReady = dataStatus;
}

void http_resetInputData() {
  memset(httpInputData, 0, sizeof(httpInputData));
}