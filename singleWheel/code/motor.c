#include "motor.h"

void motor(sint16 motor1, sint16 motor2, sint16 motor3)
{
    if (motor1 > 0)
    {
        pwm_set_duty(ATOM0_CH2_P33_11, 10000-motor1);
        gpio_low(P33_5);
    }
    else
    {
        pwm_set_duty(ATOM0_CH2_P33_11, 10000+motor1);
        gpio_high(P33_5);
    }

    if (motor2 > 0)
    {
        pwm_set_duty(ATOM0_CH4_P22_3, 10000-motor2);
        gpio_low(P22_1);
    }
    else
    {
        pwm_set_duty(ATOM0_CH4_P22_3, 10000+motor2);
        gpio_high(P22_1);
    }

    if(motor3> 3000)motor3=3000;
    if(motor3< -3000)motor3=-3000;
    if (motor3 > 0)
    {
        pwm_set_duty(ATOM0_CH7_P00_8, motor3);
        gpio_low(P00_9);
    }
    else
    {
        pwm_set_duty(ATOM0_CH7_P00_8, -motor3);
        gpio_high(P00_9);
    }
}
