#include "HexCfg.h"
#include "driver/drvPwm.h"

static byte servoCh[NUM_OF_LEGS][NUM_SERVOS_PER_LEG] = {
    // c  f  t
    {5, 6, 7},  // RF
    {2, 3, 4},  // RM
    {11, 0, 1}, // RR
    {5, 6, 7},  // LF
    {2, 3, 4},  // LM
    {11, 0, 1}  // LR
};

void servo_init()
{
  drvPwm_init();
}

void servo_setAngle(byte legNr, byte joint, int ServoAngle)
{
  byte servoNr = legNr * NUM_SERVOS_PER_LEG + joint;
  int mapped = map(ServoAngle, -900, 900, MIN_PULSE, MAX_PULSE);
  if (servoNr < NUM_SERVOS_PER_PCA)
  {
    drvPwm_set(R_DRIVER, servoCh[legNr][joint], mapped);
  }
  else
  {
    drvPwm_set(L_DRIVER, servoCh[legNr][joint], mapped);
  }
}

bool servo_setAngleImmediate(byte legNr, byte joint, int ServoAngle)
{
  if (legNr >= NUM_OF_LEGS || joint >= NUM_SERVOS_PER_LEG ||
      ServoAngle < -900 || ServoAngle > 900)
  {
    Serial.println(F("Servo: invalid immediate angle or joint."));
    return false;
  }
  int mapped = map(ServoAngle, -900, 900, MIN_PULSE, MAX_PULSE);
  return drvPwm_setImmediate(legNr < NUM_LEGS_PER_SIDE ? R_DRIVER : L_DRIVER,
                             servoCh[legNr][joint], mapped);
}

void servo_stop(byte legNr, byte joint)
{
  byte servoNr = legNr * NUM_SERVOS_PER_LEG + joint;
  if (legNr < NUM_LEGS_PER_SIDE)
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
    drvPwm_set(R_DRIVER, servoCh[legNr][FEMUR], 0);
    drvPwm_set(R_DRIVER, servoCh[legNr][TIBIA], 0);
  }
  else
  {
    drvPwm_set(L_DRIVER, servoCh[legNr][COXA], 0);
    drvPwm_set(L_DRIVER, servoCh[legNr][FEMUR], 0);
    drvPwm_set(L_DRIVER, servoCh[legNr][TIBIA], 0);
  }
}