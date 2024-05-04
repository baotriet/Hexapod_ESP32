#include "wifi.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "setup.h"

IPAddress local_IP(192, 168, 87, 200);
IPAddress gateway(192, 168, 87, 1);
IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(8, 8, 8, 8);    //optional
IPAddress secondaryDNS(8, 8, 4, 4);  //optional

void wifi_init() {
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
#ifdef DEBUG_WIFI
    Serial.println(F("STA Failed to configure"));
#endif
  }
  // Connect to Wi-Fi network
  WiFi.begin(WIFI_SSID, WIFI_PSSW);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
#ifdef DEBUG_WIFI
    Serial.println(F("Connecting to WiFi..."));
#endif  // DEBUG_WIFI
  }
#ifdef DEBUG_WIFI
  Serial.println(F("Connected to WiFi"));
  // Print ESP8266 IP address
  Serial.println(WiFi.localIP());
#endif  // DEBUG_WIFI
}