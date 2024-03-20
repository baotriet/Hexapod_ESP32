#include "footSensor.h"

Adafruit_PCF8575 pcf;

#define ARDUINO_IRQ 13  // make sure this pin is possible to make IRQ input

bool irq_flag = false;

void callback(void);

void footSensor_init(void)
{
  Wire.begin(SDA_PIN, SCL_PIN);
  pcf.begin(0x20, &Wire);
  for (int i = 0; i < 6; i++)
  {
    pcf.pinMode(i, INPUT);
  }
  // set up the interrupt pin on IRQ signal toggle
  pinMode(ARDUINO_IRQ, INPUT);
  attachInterrupt(digitalPinToInterrupt(ARDUINO_IRQ), callback, CHANGE);
}

void callback()
{
  irq_flag = true;
}

bool footSensor_getFlag(void)
{
  return irq_flag;
}

void footSensor_setFlag(bool val)
{
  irq_flag = val;
}