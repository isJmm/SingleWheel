#ifndef _CONTROL_H
#define _CONTROL_H

#include "head.h"

void Control(void);

extern float Start_Flag;
extern float M_Med_Angle, Med_Angle;
extern int16 Encoder_A,Encoder_B,Encoder_C;
extern sint16 M_Left_PWM;
extern sint16 M_Right_PWM;

#endif
