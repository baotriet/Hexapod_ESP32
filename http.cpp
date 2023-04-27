#include "http.h"

#define HTTP_PORT 80

ESP8266WebServer server(HTTP_PORT);
StaticJsonDocument<512> doc;

static bool httpDataReady = false;
static byte httpInputData[7];

void http_init()
{
    // Connect to Wi-Fi network
    WiFi.begin(WIFI_SSID, WIFI_PSSW);
    while (WiFi.status() != WL_CONNECTED)
    {
#ifdef DEBUG_HTTP
        delay(1000);
        Serial.println(F("Connecting to WiFi..."));
#endif // DEBUG_HTTP
    }
#ifdef DEBUG_HTTP
    Serial.println(F("Connected to WiFi"));
    // Print ESP8266 IP address
    Serial.println(WiFi.localIP());
#endif // DEBUG_HTTP
    // Start the server
    server.on("/values", HTTP_POST, http_parseData);
    server.begin();
}

void http_parseData()
{
    if (server.method() == HTTP_POST)
    {
        DeserializationError error = deserializeJson(doc, server.arg("plain"));
        if (error)
        {
            Serial.print(F("deserializeJson() failed: "));
            Serial.println(error.f_str());
            return;
        }
        Serial.print(F("Values received: "));
        for (int i = 1; i <= 7; i++)
        {
            String key = "value" + String(i);
            httpInputData[i] = doc[key];
            Serial.print(httpInputData[i]);
            if (i < 7)
            {
                Serial.print(F(", "));
            }
        }
        httpDataReady = true;
        Serial.println();
    }

    // Send the response
    server.send(200, "text/plain", "Values received successfully");
}

void http_update()
{
    server.handleClient();
}

byte *http_inputData()
{
    return httpInputData;
}

bool http_dataReady()
{
    return httpDataReady;
}