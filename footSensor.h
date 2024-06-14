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
 * @brief get interrupt flag
 * 
 * @return true 
 * @return false 
 */
bool footSensor_getFlag(void);

/**
 * @brief clear interrupt flag
 * 
 */
void footSensor_clrFlag(void);

/**
 * @brief check that all legs touch the ground
 * 
 * @return true 
 * @return false 
 */
bool footSensor_groundAll(void);

/**
 * @brief get the current working leg number
 * 
 */
void footSensor_getWorkingSensor(byte legNr);

/**
 * @brief check if foot sensor is activated, then stop the corresponding leg
 * 
 */
void footSensor_checkWithStop();

#endif // FOOTSENSOR_H_