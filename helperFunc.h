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
 * @brief helperFunc_smoothControl (From Zenta) -  This function makes the body rotation and translation much smoother
 *
 * @param CtrlMoveInp
 * @param CtrlMoveOut
 * @param CtrlDivider
 * @return short
 */
short helperFunc_smoothControl(short CtrlMoveInp, short CtrlMoveOut, byte CtrlDivider);

#endif /* HELPER_FUNC_H_ */