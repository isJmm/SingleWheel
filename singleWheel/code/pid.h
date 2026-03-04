#ifndef __PID_H__
#define __PID_H__

#include "head.h"

#define RANGE_PROTECT(x,max,min) ((x)<(min) ? (min) : ( (x)>(max) ? (max):(x) ))

typedef struct
{
    float SumError;  //误差累计
    float LastError;  //上一次误差
    float PrevError;  //上上次误差
    float LastData;  //上次数据
    float P;
    float I;
    float D;
    float T;  //这里不是时间，而是对积分相进行限幅
} PID_t;

typedef struct
 {
    struct //角速度环
    {
        PID_t pid;
        float out;
    } Gyro;

     struct //角度环
     {
         PID_t pid;
         float out;
     } Angle;

 }One_Wheel_Car;

extern One_Wheel_Car M;
extern One_Wheel_Car Xing;
extern One_Wheel_Car T;
extern float X_Velocity_Kp,X_Velocity_Ki,X_Velocity_out;
extern float M_Velocity_Kp,M_Velocity_Ki,M_Velocity_out;
extern float T_Velocity_Kp,T_Velocity_Ki,T_Velocity_out;
extern float EnC_Err_Lowout;
extern float b,c;

void PID_all_init(void);
void PID_init(PID_t *tem_P,float p, float i, float d, float t);
float PID_Realize(PID_t *p, float NowData, float SetData); //位置式
float X_Velocity(int encoder,float set); //带一阶低通滤波的位置式
float M_Velocity(int encoder,float set);
float T_Velocity(int encoder,float set);

#endif
