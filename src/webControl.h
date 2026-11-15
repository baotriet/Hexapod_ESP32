#ifndef WEB_CONTROL_H_
#define WEB_CONTROL_H_
#include <Arduino.h>
#include "esp_http_server.h"

// Phone Wi-Fi on the ESP32 hotspot can stall for several hundred ms (camera traffic, retries);
// 2 s tolerates that while still stopping the robot when the controller really disappears.
#define WEB_LEASE_MS 2000

// Copy a bounded value-only command across tasks; the HTTP handler must not own mutable gait
// pointers.
struct WebCommand
{
  bool enable;
  bool stop;
  bool ownershipChanged;
  int lx, ly, rx, ry;
  char action[24];
  int value;
};
void webControl_register(httpd_handle_t server);
bool webControl_take(WebCommand &command);
bool webControl_expired();
bool webControl_claimLegacy();
void webControl_status(const char *json);
#endif
