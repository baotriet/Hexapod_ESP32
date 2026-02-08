#include <Adafruit_PCF8575.h>
#include "HexCfg.h"
#include "pcf8575.h"
#include "setup.h"
#include "Wire.h"

Adafruit_PCF8575 pcf;

#define ARDUINO_IRQ 16 // make sure this pin is possible to make IRQ input

static void callback();
static bool irqFlag;

void pcf8575_init()
{
    //Wire.begin(SDA_PIN, SCL_PIN);
    if (!pcf.begin()) // 0x20
    {
        Serial.println("Failed to initialize PCF8575.");
        while (1);
    }
    for (int i = 0; i < NUM_OF_LEGS; ++i)
    {
        pcf.pinMode(i, INPUT);
    }
    pcf.digitalWrite(P10, HIGH);
    // set up the interrupt pin on IRQ signal toggle
#ifdef FOOT_SENSOR_INT_ENABLED
    pinMode(ARDUINO_IRQ, INPUT);
    irqFlag = false;
    attachInterrupt(digitalPinToInterrupt(ARDUINO_IRQ), callback, CHANGE);
#endif
}

void pcf8575_setOutput(int pinOut, bool state)
{
    pcf.digitalWrite(pinOut, state);
}

uint16_t pcf8575_getValue(uint8_t startOrder, uint8_t endOrder)
{
    uint16_t pcfState = pcf.digitalReadWord();
    uint8_t numBits = endOrder - startOrder + 1;
    uint16_t mask = (1 << numBits) - 1;
    return ((pcfState >> startOrder) & mask);
}

#ifdef FOOT_SENSOR_INT_ENABLED
/**
 * @brief Interrupt callback for PCF8575 IRQ pin.
 *
 * This minimal callback re-enables interrupts; some boards require this for
 * I2C to continue working inside an ISR.
 */
static void callback()
{
    interrupts(); // Arduino UNO seems to require that we turn on interrupts for I2C to work!
}
#endif