/* Get signal from the feet
 * 
 * License: MIT
 */

#ifndef FOOTSENSOR_H_   /* Include guard */
#define FOOTSENSOR_H_

#include <Adafruit_PCF8575.h>
#include "setup.h"

void footSensor_init(void);
bool footSensor_getFlag(void);
void footSensor_setFlag(bool val);

#endif // FOOTSENSOR_H_