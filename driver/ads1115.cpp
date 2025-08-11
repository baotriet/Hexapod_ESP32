#include <Adafruit_ADS1X15.h>
#include "Wire.h"
#include "setup.h"
#include "HexCfg.h"
#include "ads1115.h"

Adafruit_ADS1115 ads;

void ads1115_init()
{
    //Wire.begin(SDA_PIN, SCL_PIN);
    if (!ads.begin())
    {
        Serial.println("Failed to initialize ADS.");
        while (1);
    }
}

float ads1115_readVolt(short cell)
{
    unsigned int adc;
    float volt;

    adc = ads.readADC_SingleEnded(cell);
    volt = ads.computeVolts(adc);
    return volt;
}