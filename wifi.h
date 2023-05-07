#ifndef WIFI_H_   /* Include guard */
#define WIFI_H_

#include <WiFi.h>
#include "esp_http_server.h"
#include "setup.h"

#define WIFI_SSID "Zyxel_FAD7"
#define WIFI_PSSW "C3pbwWhdfYrbC"

void wifi_init();

#endif WIFI_H_