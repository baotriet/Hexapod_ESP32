/*
 * PWMServoDriver.h
 *
 *  Created on: Feb 11, 2023
 *      Author: baotriet
 */

#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include "Hex_cfg.h"
#include "setup.h"

Adafruit_PWMServoDriver pwmRight = Adafruit_PWMServoDriver(R_DRIVER);
Adafruit_PWMServoDriver pwmLeft = Adafruit_PWMServoDriver(L_DRIVER);

void PMWServoDriver_init() {
  Serial.println(F("16 test"));

  // /* Wake the device up */
  pwmRight.begin();
  pwmLeft.begin();
  pwmRight.setOscillatorFrequency(16000000);
  pwmLeft.setOscillatorFrequency(16000000);
  pwmRight.setPWMFreq(PWM_FREQ);  // Analog servos run at ~50 Hz updates
  pwmLeft.setPWMFreq(PWM_FREQ);   // Analog servos run at ~50 Hz updates
#ifdef DEBUG
  pwmRight.sleep();
  pwmLeft.sleep();
#endif
}