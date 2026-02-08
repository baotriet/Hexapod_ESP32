#include "BluetoothSerial.h"
#include "bluetooth.h"
#include "setup.h"

#define HEXPORT_RX 2
#define HEXPORT_TX 3

BluetoothSerial HexPort;

void bluetooth_init(bool needConnect) {
  HexPort.begin("ESP32-HEX");
  while (!HexPort && needConnect) {  // If the object did not initialize, then its configuration is invalid
  }
#ifdef DEBUG_BLUETOOTH
  Serial.println(F("bl done"));
#endif  // DEBUG_BLUETOOTH
}


bool bluetooth_prepareData(byte *inputData) {
  bool dataReady = 0;
  if (HexPort.available() > 0) {
    if (HexPort.read() == 'A') {
      int i = 0;
      while (HexPort.available() > 0) {
        inputData[i] = HexPort.read();
        // Serial.println(inputData[i]);
        ++i;
        if (i == 7) {
          if (HexPort.read() == 'Z') {
            dataReady = 1;
            break;
          } else {
            dataReady = 0;
            break;
          }
        }
      }
    }
  }
  return dataReady;
}

bool bluetooth_sendVoltages(float cell1, float cell2) {
  if (!HexPort) return false;
  char buf[64];
  snprintf(buf, sizeof(buf), "{\"cell1\":%.3f,\"cell2\":%.3f}", cell1, cell2);
  size_t written = HexPort.print(buf);
#ifdef DEBUG_BLUETOOTH
  Serial.print(F("Bluetooth send: "));
  Serial.println(buf);
#endif
  return (written > 0);
}