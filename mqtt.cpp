#include "mqtt.h"

#define MQTT_PORT 1883
#define MQTT_USER "hexMQTT"
#define MQTT_PSSW "baotriet11094"
#define MQTT_TOPIC "hex/command"
#define WIFI_MQTT_SINGLE_PACKET
#define MQTT_BROKER "broker.mqtt-dashboard.com"

#ifndef WIFI_MQTT_SINGLE_PACKET
const char *mqttTopic[] = {"hex/checkSum", "hex/sData_1", "hex/sData_2", "hex/Xpos", "hex/Ypos", "hex/Xpos2", "hex/Ypos2"};
int numTopic = sizeof(mqttTopic) / sizeof(mqttTopic[0]);
#endif // WIFI_MQTT_SINGLE_PACKET

WiFiClient espClient;
PubSubClient mqttClient(espClient);

static bool mqttDataReady = false;
static byte mqttInputData[7];

void mqtt_init()
{
  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(mqtt_callback);
  mqtt_connect2mqtt();
}

void mqtt_connect2mqtt()
{
  while (!mqttClient.connected())
  {
#ifdef DEBUG_WIFI_MQTT
    Serial.print(F("Attempting MQTT connection..."));
#endif // DEBUG_WIFI_MQTT
    if (mqttClient.connect("ESP8266", MQTT_USER, MQTT_PSSW))
    {
#ifdef DEBUG_WIFI_MQTT
      Serial.println(F("connected"));
#endif // DEBUG_WIFI_MQTT
       // Subscribe to MQTT topic
#ifdef WIFI_MQTT_SINGLE_PACKET
      mqttClient.subscribe(MQTT_TOPIC);
#else
      for (int i = 0; i < numTopic; i++)
      {
        mqttClient.subscribe(mqttTopic[i]);
      }
#endif // WIFI_MQTT_SINGLE_PACKET
    }
    else
    {
#ifdef DEBUG_WIFI_MQTT
      Serial.print(F("failed, rc="));
      Serial.print(mqttClient.state());
      Serial.println(F(" retrying in 5 seconds"));
#endif // DEBUG_WIFI_MQTT
      delay(3000);
    }
  }
}

// Callback function to handle incoming MQTT messages
void mqtt_callback(char *topic, byte *payload, unsigned int length)
{
#ifdef DEBUG_WIFI_MQTT
  for (int k = 0; k < length; k++)
  {
    Serial.print((char(payload[k])));
  }
  Serial.println();
#endif // DEBUG_WIFI_MQTT
#ifdef WIFI_MQTT_SINGLE_PACKET
  int i = 0;
  char *ptr = strtok((char *)payload, ";");
  while (ptr != NULL)
  {
    mqttInputData[i] = atoi(ptr);
    ptr = strtok(NULL, ";");
    ++i;
  }
  if (i == 7)
  {
    mqttDataReady = true;
  }
#else
  for (int i = 0; i < numTopic; i++)
  {
    if (strcmp(topic, mqttTopic[i]))
    {
      for (int j = 0; j < length; j++)
      {
        mqttInputData[i] |= payload[i] << (8 * i);
      }
      // if (i >= numTopic - 1)
      // {
      //   mqttDataReady = true;
      // }
      break;
    }
  }
#endif // WIFI_MQTT_SINGLE_PACKET
#ifdef DEBUG_WIFI_MQTT
  Serial.print(F("Message received on topic: "));
  Serial.println(topic);
  Serial.print(F("Payload: "));
  for (int i = 0; i < 7; i++)
  {
    Serial.print(mqttInputData[i]);
    Serial.print(F(" "));
  }
  Serial.println();
#endif // DEBUG_WIFI_MQTT
}

bool mqtt_dataReady()
{
  return mqttDataReady;
}

byte *mqtt_inputData()
{
  return mqttInputData;
}

void mqtt_update()
{
  if (!mqttClient.connected())
  {
    mqtt_connect2mqtt();
  }
  mqttClient.loop();
}

void mqtt_setDataReady(bool dataStatus)
{
  mqttDataReady = dataStatus;
}
