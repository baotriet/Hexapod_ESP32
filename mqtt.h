#ifndef MQTT_H_   /* Include guard */
#define MQTT_H_

void mqtt_init();
void mqtt_connect2mqtt();
void mqtt_callback(char* topic, byte* payload, unsigned int length);
bool mqtt_dataReady();
byte *mqtt_inputData();
void mqtt_update();
void mqtt_setDataReady(bool dataStatus);

#endif // MQTT_H_