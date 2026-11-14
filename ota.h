#ifndef HEXAPOD_OTA_H
#define HEXAPOD_OTA_H
#include "esp_http_server.h"

void ota_register(httpd_handle_t server);
void ota_update(bool allowUpdate);
bool ota_locked();

#endif
