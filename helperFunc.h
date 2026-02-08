#ifndef HELPER_FUNC_H_
#define HELPER_FUNC_H_

#include "setup.h"

/**
 * @brief Get the sinus and cosinus from the angle +/- multiple circles
 *
 * @param AngleDeg1 input angle in degrees
 * @return short*   sin and cos value of input angle
 */
short *helperFunc_getSinCos(short AngleDeg1);

/**
 * @brief Get the Arc Cos
 *
 * @param cos4      cos value of angle
 * @return short    angle in radian
 */
short helperFunc_getArcCos(short cos4);

/**
 * @brief calculate integer square root
 *
 * @param n
 * @return unsigned long
 */
unsigned long helperFunc_isqrt32(unsigned long n);

/**
 * @brief Simplyfied ArcTan2 function based on fixed point ArcCos
 *
 * @param AtanX     input X
 * @param AtanY     Input Y
 * @return long     Atan and XYhyp2
 */
long *helperFunc_getATan2(short AtanX, short AtanY);

/**
 * @brief helperFunc_smoothUpdate -  This function makes the hex move much smoother
 *
 * @param currentVal    current value
 * @param targetVal     target value
 * @param baseDiv       divider to smooth the value
 * @param threshold     minimum gap between currentVal and targetVal
 * @return updated value
 */
short helperFunc_smoothBasedVal(short currentVal, short targetVal, byte baseDiv, byte threshold);

/**
 * @brief Reverse command direction from 2 joysticks left and right
 * 
 * @param lx    left x command
 * @param ly    left y command
 * @param rx    right x command
 * @param ry    right y command
 */
void helperFunc_reverseCmd(byte *lx, byte *ly, byte *rx, byte *ry);


#endif /* HELPER_FUNC_H_ */