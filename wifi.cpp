#include "wifi.h"

#define WIFI_SSID "Zyxel_FAD7"
#define WIFI_PSSW "C3pbwWhdfYrbC"

#define MQTT_PORT 1883
#define MQTT_USER "hexMQTT"
#define MQTT_PSSW "baotriet11094"
#define MQTT_TOPIC "hex/command"

WiFiClient espClient;
PubSubClient mqttClient(espClient);

static bool dataReady = 0;
static byte inputData[7];

void wifi_init() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PSSW);

  IPAddress myIP = WiFi.localIP();
#ifdef DEBUG_WIFI
  Serial.println(myIP);
#endif  // DEBUG_WIFI

  mqttClient.setServer("broker.mqtt-dashboard.com", MQTT_PORT);
  mqttClient.setCallback(wifi_callback);
  wifi_connect2mqtt();
}

void wifi_connect2mqtt() {
  while (!mqttClient.connected()) {
#ifdef DEBUG_WIFI
    Serial.print("Attempting MQTT connection...");
#endif  // DEBUG_WIFI
    if (mqttClient.connect("ESP8266", MQTT_USER, MQTT_PSSW)) {
#ifdef DEBUG_WIFI
      Serial.println("connected");
#endif  // DEBUG_WIFI
      // Subscribe to MQTT topic
      mqttClient.subscribe(MQTT_TOPIC);
    } else {
#ifdef DEBUG_WIFI
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" retrying in 5 seconds");
#endif  // DEBUG_WIFI
      delay(5000);
    }
  }
}

// Callback function to handle incoming MQTT messages
void wifi_callback(char* topic, byte* payload, unsigned int length) {
#ifdef DEBUG_WIFI
  Serial.print("Message received on topic: ");
  Serial.println(topic);
  Serial.print("Payload: ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
    //Serial.print(",");
  }
  Serial.println();
#else
  if ((char)payload[0] == 'A') {
    int i = 0;
    while (i < length) {
      inputData[i] = (char)payload[i++];
      i++;
      if (i == 7) {
        if ((char)payload[i++] == 'Z') {
          dataReady = 1;
          break;
        } else {
          dataReady = 0;
          break;
        }
      }
    }
  }
#endif  // DEBUG_WIFI
}

bool wifi_dataReady() {
  return dataReady;
}

byte* wifi_subsData() {
  return inputData;
}

void wifi_update(){
  if(!mqttClient.connected()){
    wifi_connect2mqtt();
  }
  mqttClient.loop();
}