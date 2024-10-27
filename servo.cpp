#include "HexCfg.h"
#include "drvPwm.h"

static byte servoCh[NUM_OF_LEGS][NUM_SERVOS_PER_LEG] = {
    // c  f  t
    {0, 1, 2}, // RR
    {7, 6, 5}, // RM
    {8, 9, 10}, // RF
    {13, 14, 15},  // LR
    {10, 9, 8}, // LM
    {7, 6, 5} // LF
    
};

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