#include "wifi.h"

void wifi_init()
{
    // Connect to Wi-Fi network
    WiFi.begin(WIFI_SSID, WIFI_PSSW);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
#ifdef DEBUG_WIFI
        Serial.println(F("Connecting to WiFi..."));
#endif // DEBUG_WIFI
    }
#ifdef DEBUG_WIFI
    Serial.println(F("Connected to WiFi"));
    // Print ESP8266 IP address
    Serial.println(WiFi.localIP());
#endif // DEBUG_WIFI
}