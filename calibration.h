#ifndef HEXAPOD_CALIBRATION_H
#define HEXAPOD_CALIBRATION_H

#include <Arduino.h>
#include "esp_http_server.h"

void calibration_init();
void calibration_register(httpd_handle_t server);
bool calibration_busy();
bool calibration_update(bool allowCalibration, void (*move)(byte, byte, short));
short calibration_offset(byte leg, byte joint);

#endif
