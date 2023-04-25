#ifndef PID_H_   /* Include guard */
#define PID_H_

#include <PID_v2.h>
#include "mpu6050.h"
#include "setup.h"

void pid_init();
const double *pid_update(double *output, int size);

#endif // PID_H_