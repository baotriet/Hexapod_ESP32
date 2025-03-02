/* Get tilt angles on X and Y, and rotation angle on Z
 * Angles are given in degrees
 * 
 * License: MIT
 */

#ifndef MPU6050_H_   /* Include guard */
#define MPU6050_H_

#include "stdbool.h"
#include "setup.h"

void mpu6050_init(bool isInit);
float mpu6050_getAngle(char Axis);
void mpu6050_update();

#endif // MPU6050_H_