#ifndef SERVO_H /* Include guard */
#define SERVO_H

#include "setup.h"

/**
 * @brief Initialize for servo module
 * 
 */
void servo_init(void);

/**
 * @brief This function simplifies servo control
 *
 * @param legNr         Order number of a leg
 * @param joint         type of joint
 * @param ServoAngle    desired servo angle (unit: pulse, between MIN_PULSE and MAX_PULSE)
 */
void servo_setAngle(byte legNr, byte joint, int ServoAngle);

/**
 * @brief stop servo by sending zero pulse
 * 
 * @param legNr     Order number of a leg
 * @param joint     type of joint
 */
void servo_stop(byte legNr, byte joint);

/**
 * @brief stop the selected leg
 * 
 * @param legNr     Order number of a leg
 */
void servo_stopLeg(byte legNr);

#endif // SERVO_H