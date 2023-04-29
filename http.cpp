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
        delay(1000);
#ifdef DEBUG_HTTP
        Serial.println(F("Connecting to WiFi..."));
#endif // DEBUG_HTTP
    }
#ifdef DEBUG_HTTP
    Serial.println(F("Connected to WiFi"));
    // Print ESP8266 IP address
    Serial.println(WiFi.localIP());
#endif // DEBUG_HTTP
    // Start the server
    server.on("/values/", HTTP_POST, http_parseData);
    server.begin();
}

void http_parseData()
{
    if (server.method() == HTTP_POST)
    {
        DeserializationError error = deserializeJson(doc, server.arg("plain"));
        if (error)
        {
#ifdef DEBUG_HTTP
            Serial.print(F("deserializeJson() failed: "));
            Serial.println(error.f_str());
#endif // DEBUG_HTTP
            return;
        }
#ifdef DEBUG_HTTP
        Serial.print(F("Values received: "));
#endif // DEBUG_HTTP
        for (int i = 0; i < 7; i++)
        {
            String key = "value" + String(i);
            httpInputData[i] = doc[key];
#ifdef DEBUG_HTTP
            Serial.print(httpInputData[i]);
#endif // DEBUG_HTTP
        }
#ifdef DEBUG_HTTP
        Serial.println();
#endif // DEBUG_HTTP
    }

    // Send the response
    server.send(200, "text/plain", "Values received successfully");
    // server.close();
    httpDataReady = true;
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

void http_setDataReady(bool dataStatus)
{
    httpDataReady = dataStatus;
}

void http_resetInputData()
{
    memset(httpInputData, 0, sizeof(httpInputData));
}