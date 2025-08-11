#include "HexCfg.h"
#include "footSensor.h"
#include "servo.h"
#include "setup.h"
#include "driver/pcf8575.h"

static uint8_t sensorMask;

void footSensor_init(void)
{
  sensorMask = 0;
}

bool footSensor_isAnyLegTriggered(void)
{
  int retval = false;
  for(int legNr = 0; legNr < NUM_OF_LEGS; ++legNr)
  {
    if(sensorMask & (1 << legNr))
    {
      retval = true;
      break;
    }
  }
  return retval;
}

void footSensor_update()
{
  sensorMask = pcf8575_getValue(0, NUM_OF_LEGS-1);
}

bool footSensor_isAllLegsTriggered(void)
{
  int retval = false;
  for(int legNr = 0; legNr < NUM_OF_LEGS; ++legNr)
  {
    if(!(sensorMask & (1 << legNr)))
    {
      retval = true;
      break;
    }
  }
  return retval;
}

bool footSensor_isTriggered(byte legNr)
{
  return (sensorMask & (1 << legNr)) != 0;
}

uint8_t footSensor_getSensorMask()
{
  return sensorMask;
}

void footSensor_setSensorMask(uint8_t mask)
{
  sensorMask = mask;
}