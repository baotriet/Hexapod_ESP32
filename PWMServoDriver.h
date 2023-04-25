/*
 * PWMServoDriver.h
 *
 *  Created on: Feb 11, 2023
 *      Author: baotriet
 */

#ifndef PWMSERVODRIVER_H_   /* Include guard */
#define PWMSERVODRIVER_H_

#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include "Hex_cfg.h"
#include "setup.h"

void PMWServoDriver_init();
void PMWServoDriver_set(uint16_t addr, byte channel, int mapAngle);
void PMWServoDriver_goSleep();

#endif // PWMSERVODRIVER_H_