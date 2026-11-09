#ifndef PID_H_ /* Include guard */
#define PID_H_

#include "setup.h"

void pid_init();
// Corrections are in degrees; body kinematics converts them to tenths of a degree.
const float *pid_update(float *output, int size);

#endif // PID_H_