#include "driver/drvPwm.h"
#include <Adafruit_PWMServoDriver.h>
#include "HexCfg.h"

Adafruit_PWMServoDriver pwmRight = Adafruit_PWMServoDriver(R_DRIVER);
Adafruit_PWMServoDriver pwmLeft = Adafruit_PWMServoDriver(L_DRIVER);

void drvPwm_goSleep()
{
  pwmRight.sleep();
  pwmLeft.sleep();
}

void drvPwm_set(uint16_t addr, byte channel, int pwmPulse)
{
  if (R_DRIVER == addr)
  {
    pwmRight.setPWM(channel, 0, pwmPulse);
  }
  else if (L_DRIVER == addr)
  {
    pwmLeft.setPWM(channel, 0, pwmPulse);
  }
}

void drvPwm_init()
{
#ifdef DEBUG_PWM
  Serial.println(F("16 test"));
#endif // DEBUG_PWM
  // /* Wake the device up */
  pwmRight.begin();
  pwmLeft.begin();
  pwmRight.setOscillatorFrequency(R_Frequency);
  pwmLeft.setOscillatorFrequency(L_Frequency);
  pwmRight.setPWMFreq(PWM_FREQ); // Analog servos run at ~50 Hz updates
  pwmLeft.setPWMFreq(PWM_FREQ);  // Analog servos run at ~50 Hz updates
}