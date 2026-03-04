
#include "head.h"

float Start_Flag;
float M_Med_Angle = 3.0, Med_Angle = -0.2;
int16 Encoder_A,Encoder_B,Encoder_C;
sint16 M_Left_PWM = 0;
sint16 M_Right_PWM = 0;

void Control(void)
{
    static short time;
    time ++;

    if(0 == (time % 2))  //每2ms执行一次
    {
        ADIS16505_task();
        M.Gyro.out = PID_Realize(&M.Gyro.pid, ADIS16505_.gx * 1000, M.Angle.out);
        Xing.Gyro.out = PID_Realize(&Xing.Gyro.pid, -ADIS16505_.gy * 1000, Xing.Angle.out);
        T.Gyro.out = PID_Realize(&T.Gyro.pid, ADIS16505_.gz * 1000, T.Angle.out);
        M.Gyro.out = fclip(M.Gyro.out, -10000, 10000);
        T.Gyro.out = fclip(T.Gyro.out, -10000, 10000);

        M_Left_PWM  = M.Gyro.out + T.Gyro.out;
        M_Right_PWM = -M.Gyro.out + T.Gyro.out;

        M_Left_PWM    = fclip(M_Left_PWM, -10000, 10000);                  //PWM限幅
        M_Right_PWM   = fclip(M_Right_PWM, -10000, 10000);                 //PWM限幅
        Xing.Gyro.out = fclip(Xing.Gyro.out, -3000, 3000);                 //PWM限幅

        if(fabs (M_eulerAngle_roll) > 30 || fabs (M_eulerAngle_pitch) > 20)   Start_Flag = 0;
        if(Start_Flag == 0)
        {
            DIN_ON;
            motor(0, 0, 0);

            speed = 0;

            M.Gyro.pid.SumError = 0;
            Xing.Gyro.pid.SumError = 0;
            T.Gyro.pid.SumError = 0;
        }
        else
        {
            DIN_OFF;
            motor(-M_Left_PWM, -M_Right_PWM, Xing.Gyro.out);
        }
    }

    if(0 == (time % 5))      //每5ms控制一次
    {
        M.Angle.out = PID_Realize(&M.Angle.pid, M_eulerAngle_roll, (M_Med_Angle + Dynamic_zero_set + M_Velocity_out));
        Xing.Angle.out = PID_Realize(&Xing.Angle.pid, M_eulerAngle_pitch, Med_Angle + X_Velocity_out);
        T.Angle.out = PID_Realize(&T.Angle.pid, angleUsed, 0);
    }

    if(0 == (time % 10))      //每10ms执行一次
    {
        Encoder_A = encoder_get_count(TIM2_ENCODER); //左动量轮
        Encoder_B = -encoder_get_count(TIM6_ENCODER); //右动量轮
        Encoder_C = -encoder_get_count(TIM3_ENCODER); //行进轮

        encoder_get();
        encoder_clear_count(TIM2_ENCODER);

        encoder_clear_count(TIM6_ENCODER);
        encoder_clear_count(TIM3_ENCODER);

        M_Velocity_out = M_Velocity((Encoder_A + Encoder_B), 0);
        X_Velocity_out = X_Velocity(Encoder_C, speed);

        time = 0;
    }
}



