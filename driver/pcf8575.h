#ifndef PCF8575_H_   /* Include guard */
#define PCF8575_H_

#define WIFI_SSID "Stofa07897"
#define WIFI_PSSW "gav47titte05"

#include <stdint.h>

/**
 * @brief Initialize pcf8575 module
 * 
 */
void pcf8575_init();

/**
 * @brief Set state of an output pin
 * 
 * @param pinOut    pin needs to be set 
 * @param state     desired state
 */
void pcf8575_setOutput(int pinOut, bool state);

/**
 * @brief Get the value of specific bits
 * 
 * @param startOrder the first bit needed to get
 * @param endOrder the last bit need to get
 * @return the value of bits from the first bit to the last one
 */
uint16_t pcf8575_getValue(uint8_t startOrder, uint8_t endOrder);

#endif //PCF8575_H_