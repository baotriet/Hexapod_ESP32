/* Get signal from the feet
 * 
 * License: MIT
 */

#ifndef FOOTSENSOR_H_   /* Include guard */
#define FOOTSENSOR_H_

#include "setup.h"

/**
 * @brief init function
 * 
 */
void footSensor_init(void);

/**
 * @brief check if any leg touches ground
 * 
 * @return true if there is any leg touching ground 
 * @return false otherwise
 */
bool footSensor_isAnyLegTriggered(void);

/**
 * @brief check that all legs touch the ground
 * 
 * @return true if all legs touch ground
 * @return false otherwise
 */
bool footSensor_isAllLegsTriggered(void);

/**
 * @brief check if the leg touched ground
 * 
 * @param legNr the leg needing to be checked 
 * @return true if the leg touched ground
 * @return false otherwise
 */
bool footSensor_isTriggered(byte legNr);

/**
 * @brief get the number of grounded legs
 * 
 * @return the number of grounded legs
 */
uint8_t footSensor_getSensorMask();

/**
 * @brief set the number of grounded legs
 * 
 * @param mask the number of grounded legs
 */
void footSensor_setSensorMask(uint8_t mask);

#endif // FOOTSENSOR_H_