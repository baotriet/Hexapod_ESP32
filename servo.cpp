#include "HexCfg.h"
#include "drvPwm.h"

#define NUM_OF_LEG 6
#define NUM_SERVOS_PER_LEG 3
#define NUM_OF_SERVO 18

static byte servoCh[NUM_OF_LEG][NUM_SERVOS_PER_LEG] = {
  // c  f  t
    {6, 7, 8}, // RR
    {3, 4, 5}, // RM
    {0, 1, 2}, // RF
    {0, 1, 2}, // LR
    {3, 4, 5}, // LM
    {6, 7, 8}  // LF
};

void
servo_setAngle(byte legNr, byte joint, int ServoAngle)
{
    unsigned int servoNr = legNr * NUM_SERVOS_PER_LEG + joint;
    if (servoNr < 9)
    {
        drvPwm_set(R_DRIVER, servoCh[legNr][joint], map(ServoAngle, -900, 900, MIN_PULSE, MAX_PULSE));
    }
    else
    {
        drvPwm_set(L_DRIVER, servoCh[legNr][joint], map(ServoAngle, -900, 900, MIN_PULSE, MAX_PULSE));
    }
}