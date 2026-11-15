#include <Adafruit_ADS1X15.h>
#include "Wire.h"
#include "setup.h"
#include "HexCfg.h"
#include "ads1115.h"
#include <math.h>
#include "serialOutput.h"

Adafruit_ADS1115 ads;
static uint32_t conversionStarted;
static bool converting;

static bool readRegister(byte reg, uint16_t &value)
{
  Wire.beginTransmission(ADS1X15_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0 ||
      Wire.requestFrom(uint8_t(ADS1X15_ADDRESS), uint8_t(2)) != 2)
  {
    return false;
  }
  value = uint16_t(Wire.read()) << 8;
  value |= uint16_t(Wire.read());
  return true;
}

void ads1115_init()
{
  converting = false;
  // Wire.begin(SDA_PIN, SCL_PIN);
  if (!ads.begin())
  {
    Serial.println("Failed to initialize ADS.");
    while (1)
    {
    }
  }
}

bool ads1115_start(short cell)
{
  if (cell < 0 || cell > 3 || converting)
  {
    Serial.println(F("Battery ADC: invalid or overlapping conversion."));
    return false;
  }
  uint16_t config = ADS1X15_REG_CONFIG_OS_SINGLE | MUX_BY_CHANNEL[cell] |
                    ADS1X15_REG_CONFIG_PGA_6_144V | ADS1X15_REG_CONFIG_MODE_SINGLE |
                    RATE_ADS1115_128SPS | ADS1X15_REG_CONFIG_CQUE_NONE;
  Wire.beginTransmission(ADS1X15_ADDRESS);
  Wire.write(ADS1X15_REG_POINTER_CONFIG);
  Wire.write(uint8_t(config >> 8));
  Wire.write(uint8_t(config));
  if (Wire.endTransmission() != 0)
  {
    return false;
  }
  conversionStarted = millis();
  converting = true;
  return true;
}

AdcReadStatus ads1115_poll(float &voltage)
{
  if (!converting)
  {
    Serial.println(F("Battery ADC: no conversion pending."));
    return AdcReadStatus::Error;
  }
  uint32_t elapsed = uint32_t(millis() - conversionStarted);
  if (elapsed < 8)
  {
    return AdcReadStatus::Pending;
  }
  uint16_t config;
  if (elapsed >= 50 || !readRegister(ADS1X15_REG_POINTER_CONFIG, config))
  {
    converting = false;
    return AdcReadStatus::Error;
  }
  if (!(config & ADS1X15_REG_CONFIG_OS_NOTBUSY))
  {
    return AdcReadStatus::Pending;
  }
  uint16_t raw;
  converting = false;
  if (!readRegister(ADS1X15_REG_POINTER_CONVERT, raw))
  {
    return AdcReadStatus::Error;
  }
  voltage = roundf(ads.computeVolts(int16_t(raw)) * 100.0f) / 100.0f;
  return AdcReadStatus::Ready;
}