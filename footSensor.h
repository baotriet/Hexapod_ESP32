/* Get signal from the feet
 *
 * License: MIT
 */

#ifndef FOOTSENSOR_H_ /* Include guard */
#define FOOTSENSOR_H_

#include "setup.h"
#include "footSensorCfg.h"

/**
 * @brief init function
 *
 */
void footSensor_init(void);
void footSensor_update(void);
void footSensor_report(void);
bool footSensor_mappingValid(void);
bool footSensor_isRawTriggered(byte legNr);
bool footSensor_isReleasedStable(byte legNr);

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
 * @brief Get the debounced contact bitmask indexed by configured leg number.
 *
 * @return Contact bitmask, not a count.
 */
uint8_t footSensor_getSensorMask();

/**
 * @brief Override the contact mask until the next hardware sample (diagnostics).
 *
 * @param mask Contact bitmask.
 */
void footSensor_setSensorMask(uint8_t mask);

#endif // FOOTSENSOR_H_