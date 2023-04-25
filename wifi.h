#ifndef WIFI_H_   /* Include guard */
#define WIFI_H_

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266WebServer.h>

void wifi_init();
void wifi_connect2mqtt();
void wifi_callback(char* topic, byte* payload, unsigned int length);
bool wifi_dataReady();
byte *wifi_subsData();
void wifi_update();

#endif // WIFI_H_