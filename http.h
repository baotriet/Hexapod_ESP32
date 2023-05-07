#ifndef HTTP_H_   /* Include guard */
#define HTTP_H_

#include <ArduinoJson.h>
#include <WiFi.h>
#include "esp_http_server.h"
#include "setup.h"
#include "camera.h"

void http_init();
byte *http_inputData();
bool http_dataReady();
void http_setDataReady(bool dataStatus);
void http_resetInputData();

#endif // HTTP_H_