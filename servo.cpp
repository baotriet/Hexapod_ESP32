#include "HexCfg.h"
#include "driver/drvPwm.h"

static byte servoCh[NUM_OF_LEGS][NUM_SERVOS_PER_LEG] = {
    // c  f  t
    {0, 1, 2}, // RR
    {3, 4, 5}, // RM
    {6, 7, 8}, // RF
    {0, 1, 2}, // LR
    {3, 4, 5}, // LM
    {6, 7, 8}  // LF
    
};

void servo_init()
{
    drvPwm_init();
}

void servo_setAngle(byte legNr, byte joint, int ServoAngle)
{
    unsigned int servoNr = legNr * NUM_SERVOS_PER_LEG + joint;
    if (servoNr < NUM_SERVOS_PER_PCA)
    {
        drvPwm_set(R_DRIVER, servoCh[legNr][joint], map(ServoAngle, -900, 900, MIN_PULSE, MAX_PULSE));
    }
    else
    {
        drvPwm_set(L_DRIVER, servoCh[legNr][joint], map(ServoAngle, -900, 900, MIN_PULSE, MAX_PULSE));
    }
}

void servo_stop(byte legNr, byte joint)
{
    unsigned int servoNr = legNr * NUM_SERVOS_PER_LEG + joint;
    if (servoNr < NUM_LEGS_PER_SIDE)
    {
        drvPwm_set(R_DRIVER, servoCh[legNr][joint], 0);
    }
    else
    {
        drvPwm_set(L_DRIVER, servoCh[legNr][joint], 0);
    }
}

void servo_stopLeg(byte legNr)
{
    if (legNr < NUM_LEGS_PER_SIDE)
    {
        drvPwm_set(R_DRIVER, servoCh[legNr][COXA], 0);
        drvPwm_set(R_DRIVER, servoCh[legNr][FERMUR], 0);
        drvPwm_set(R_DRIVER, servoCh[legNr][TIBIA], 0);
    }
    else
    {
        drvPwm_set(L_DRIVER, servoCh[legNr][COXA], 0);
        drvPwm_set(L_DRIVER, servoCh[legNr][FERMUR], 0);
        drvPwm_set(L_DRIVER, servoCh[legNr][TIBIA], 0);
    }
}