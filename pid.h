#ifndef PID_H_   /* Include guard */
#define PID_H_

#include "setup.h"

void pid_init();
const float *pid_update(float *output, int size);

#endif // PID_H_