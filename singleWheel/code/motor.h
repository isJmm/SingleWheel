/*
 * motor.h
 *
 *  Created on: 2023Äê5ÔÂ18ÈÕ
 *      Author: lan
 */

#ifndef CODE_MOTOR_H_
#define CODE_MOTOR_H_

#include "head.h"

#define DIN_ON      gpio_low(P33_4),gpio_low(P23_1)
#define DIN_OFF     gpio_high(P33_4),gpio_high(P23_1)

void motor(sint16 motor1, sint16 motor2, sint16 motor3);

#endif /* CODE_MOTOR_H_ */
