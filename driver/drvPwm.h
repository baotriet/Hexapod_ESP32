/*
 * drvPwm.h
 *
 *  Created on: Feb 11, 2023
 *      Author: baotriet
 */

#ifndef DRV_PWM_H_ /* Include guard */
#define DRV_PWM_H_

#include "setup.h"
#if BUILDCFG_MOTION_TIMING_ENABLED
#include "motionTiming.h"
PwmTiming &drvPwm_timing();
#endif

/**
 * @brief init function
 *
 */
void drvPwm_init();

/**
 * @brief set the PWM to control the servo
 *
 * @param addr      I2C address of PCA9685
 * @param channel   connected channel of PCA9685
 * @param pwmPulse  mapped angle
 */
void drvPwm_set(uint16_t addr, byte channel, int pwmPulse);
// Immediate writes report I2C success and reject calls during a batch.
bool drvPwm_setImmediate(uint16_t addr, byte channel, int pwmPulse);
void drvPwm_beginBatch();
void drvPwm_endBatch();

/**
 * @brief put PCA9685 to sleep
 *
 */
void drvPwm_goSleep();

#endif // DRV_PWM_H_