#include "PWMServoDriver.h"

Adafruit_PWMServoDriver pwmRight = Adafruit_PWMServoDriver(R_DRIVER);
Adafruit_PWMServoDriver pwmLeft = Adafruit_PWMServoDriver(L_DRIVER);

void PMWServoDriver_goSleep() {
  pwmRight.sleep();
  pwmLeft.sleep();
}

void PMWServoDriver_set(uint16_t addr, byte channel, int mapAngle) {
  if (R_DRIVER == addr)
    pwmRight.setPWM(channel, 0, mapAngle);
  else if (L_DRIVER == addr)
    pwmLeft.setPWM(channel, 0, mapAngle);
}

void PMWServoDriver_init() {
#ifdef DEBUG_PWM
  Serial.println(F("16 test"));
#endif  // DEBUG_PWM

  // /* Wake the device up */
  pwmRight.begin();
  pwmLeft.begin();
  pwmRight.setOscillatorFrequency(16000000);
  pwmLeft.setOscillatorFrequency(16000000);
  pwmRight.setPWMFreq(PWM_FREQ);  // Analog servos run at ~50 Hz updates
  pwmLeft.setPWMFreq(PWM_FREQ);   // Analog servos run at ~50 Hz updates
  //#ifdef DEBUG
  //pwmRight.sleep();
  //pwmLeft.sleep();
  //#endif
}