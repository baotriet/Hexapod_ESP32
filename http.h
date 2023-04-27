#ifndef HTTP_H_   /* Include guard */
#define HTTP_H_

#include <ArduinoJson.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include "setup.h"

void http_init();
void http_update();
void http_parseData();
byte *http_inputData();
bool http_dataReady();

#endif // HTTP_H_