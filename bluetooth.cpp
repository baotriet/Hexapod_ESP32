#include "bluetooth.h"

#define HEXPORT_RX 2
#define HEXPORT_TX 3

SoftwareSerial HexPort = SoftwareSerial(HEXPORT_RX, HEXPORT_TX);

void bluetooth_init(bool needConnect) {
  // Define pin modes for TX and RX
  pinMode(HEXPORT_RX, INPUT);
  pinMode(HEXPORT_TX, OUTPUT);
  HexPort.begin(9600);
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