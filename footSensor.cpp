#include <Adafruit_PCF8575.h>
#include "footSensor.h"
#include "setup.h"

Adafruit_PCF8575 pcf;

#define ARDUINO_IRQ 13 // make sure this pin is possible to make IRQ input

bool irqFlag;
int workingSensor;

void callback(void);

void footSensor_init(void)
{
  Wire.begin(SDA_PIN, SCL_PIN);
  pcf.begin(0x20, &Wire);
  for (int i = 0; i < 6; i++)
  {
    pcf.pinMode(i, INPUT);
  }
  irqFlag = false;

  // set up the interrupt pin on IRQ signal toggle
  pinMode(ARDUINO_IRQ, INPUT);
#ifdef FOOT_SENSOR_INT_ENABLED
  attachInterrupt(digitalPinToInterrupt(ARDUINO_IRQ), callback, CHANGE);
#endif
}

void callback()
{
  if (irqFlag)
    return; // we are already handling an irq so don't collide!
  irqFlag = true;
  interrupts(); // Arduino UNO seems to require that we turn on interrupts for I2C to work!
  if(pcf.digitalRead(workingSensor))
  {

  }
}

bool footSensor_getFlag(void)
{
  return irqFlag;
}

void footSensor_clrFlag(void)
{
  irqFlag = false;
}

bool footSensor_checkAllSensor(void)
{
  byte allSensorState = (pcf.digitalReadWord() >> 0) & 0xFF;
  // Serial.println(allSensorState);
  if (allSensorState == B111111)
  {
    return true;
  }
  return false;
}

void footSensor_getWorkingSensor(byte legNr)
{
  workingSensor = legNr;
}

bool footSensor_checkSingleSensor(byte legNr)
{

}