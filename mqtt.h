#ifndef MQTT_H_   /* Include guard */
#define MQTT_H_

#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266WebServer.h>
#include "setup.h"

void mqtt_init();
void mqtt_connect2mqtt();
void mqtt_callback(char* topic, byte* payload, unsigned int length);
bool mqtt_dataReady();
byte *mqtt_inputData();
void mqtt_update();

#endif // MQTT_H_