#include "wifi.h"

#define WIFI_SSID "Zyxel_FAD7"
#define WIFI_PSSW "C3pbwWhdfYrbC"

#define MQTT_PORT 1883
#define MQTT_USER "hexMQTT"
#define MQTT_PSSW "baotriet11094"
#define MQTT_TOPIC "hex/command"
// #define WIFI_MQTT_SINGLE_PACKET
#define MQTT_BROKER "broker.mqtt-dashboard.com"

#ifndef WIFI_MQTT_SINGLE_PACKET
const char *mqttTopic[] = {"hex/checkSum", "hex/sData_1", "hex/sData_2", "hex/Xpos", "hex/Ypos", "hex/Xpos2", "hex/Ypos2"};
int numTopic = sizeof(mqttTopic) / sizeof(mqttTopic[0]);
#endif // WIFI_MQTT_SINGLE_PACKET

WiFiClient espClient;
PubSubClient mqttClient(espClient);

static bool dataReady = 0;
static byte inputData[7]};

void wifi_init()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PSSW);

#ifdef DEBUG_WIFI_MQTT
  Serial.println(myIP);
#endif // DEBUG_WIFI_MQTT

  mqttClient.setServer(MQTT_BROKER, MQTT_PORT);
  mqttClient.setCallback(wifi_callback);
  wifi_connect2mqtt();
}

void wifi_connect2mqtt()
{
  while (!mqttClient.connected())
  {
#ifdef DEBUG_WIFI_MQTT
    Serial.print("Attempting MQTT connection...");
#endif // DEBUG_WIFI_MQTT
    if (mqttClient.connect("ESP8266", MQTT_USER, MQTT_PSSW))
    {
#ifdef DEBUG_WIFI_MQTT
      Serial.println("connected");
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
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying in 5 seconds");
#endif // DEBUG_WIFI_MQTT
      delay(3000);
    }
  }
}

// Callback function to handle incoming MQTT messages
void wifi_callback(char *topic, byte *payload, unsigned int length)
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
    inputData[i] = atoi(ptr);
    ptr = strtok(NULL, ";");
    ++i;
  }
  if (i == 7)
  {
    dataReady = 1;
  }
  else
  {
    dataReady = 0;
  }
#else
  for (int i = 0; i < numTopic; i++)
  {
    if (strcmp(topic, mqttTopic[i]))
    {
      for (int j = 0; j < length; j++)
      {
        inputData[i] |= payload[i] << (8 * i);
      }
      // if (i >= numTopic - 1)
      // {
      //   dataReady = true;
      // }
      break;
    }
  }
#endif // WIFI_MQTT_SINGLE_PACKET
#ifdef DEBUG_WIFI_MQTT
  Serial.print("Message received on topic: ");
  Serial.println(topic);
  Serial.print("Payload: ");
  for (int i = 0; i < 7; i++)
  {
    Serial.print(inputData[i]);
    Serial.print(" ");
  }
  Serial.println();
#endif // DEBUG_WIFI_MQTT
}

bool wifi_dataReady()
{
  return dataReady;
}

byte *wifi_subsData()
{
  return inputData;
}

void wifi_update()
{
  if (!mqttClient.connected())
  {
    wifi_connect2mqtt();
  }
  mqttClient.loop();
}