#ifndef SERVO_H /* Include guard */
#define SERVO_H

#include "setup.h"

/**
 * @brief This function simplifies servo control
 *
 * @param legNr         Order number of a leg
 * @param joint         type of joint
 * @param ServoAngle    desired servo angle (unit: pulse, between MIN_PULSE and MAX_PULSE)
 */
void servo_setAngle(byte legNr, byte joint, int ServoAngle);

#endif // SERVO_H