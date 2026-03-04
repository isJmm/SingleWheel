#include "pid.h"

One_Wheel_Car M;
One_Wheel_Car Xing;
One_Wheel_Car T;
float X_Velocity_Kp,X_Velocity_Ki,X_Velocity_out;
float M_Velocity_Kp,M_Velocity_Ki,M_Velocity_out;
float T_Velocity_Kp,T_Velocity_Ki,T_Velocity_out;

void PID_all_init(void)
{
    //动量轮
    PID_init(&M.Gyro.pid,  8.0, 0.5, 2, 3000);
    PID_init(&M.Angle.pid, 110, 0, 0, 100);
    M_Velocity_Kp =  0.025;
    M_Velocity_Ki =  0.0001;
    M_Velocity_out = 0;//速度环

    //行进轮
    PID_init(&Xing.Gyro.pid, 1.2, 0.06, 0, 2000);
    PID_init(&Xing.Angle.pid, 80, 0, 2, 100);
    X_Velocity_Kp = 0.005;
    X_Velocity_Ki = 0;
    X_Velocity_out =0;

    //转向
    PID_init(&T.Gyro.pid, -20, 0, 0, 100);//角速度环
    PID_init(&T.Angle.pid, 25, 0, 0, 100);//角度环
    T_Velocity_Kp = 0;
    T_Velocity_Ki = 0;
    T_Velocity_out = 0;//速度环
}



void PID_init(PID_t *tem_P,float p, float i, float d, float t)
{
    tem_P->LastData = 0;
    tem_P->LastError = 0;
    tem_P->PrevError = 0;
    tem_P->SumError = 0;

    tem_P->P = p;
    tem_P->I = i;
    tem_P->D = d;
    tem_P->T = t;
}


//位置式PID
float PID_Realize(PID_t *p, float NowData, float SetData)
{
    //当前误差，定义为寄存器变量，只能用于整型和字符型变量，提高运算速度
    float iError,   // 当前误差
          Realize;  // 最后得出的实际增量

    iError = SetData - NowData; // 计算当前误差
    p->SumError += p->I * iError;   // 误差积分
    p->SumError = RANGE_PROTECT(p->SumError, p->T, -(p->T));

    Realize = ( (p->P) * iError
                     + p->SumError
                     + p->D * (iError - p->LastError) );

    p->PrevError = p->LastError;    // 更新前次误差
    p->LastError = iError;         // 更新上次误差
    p->LastData  = NowData;        // 更新上次数据

    return Realize; // 返回实际值
}

float b = 0,c = 0;
float EnC_Err_Lowout = 0;
float X_Velocity(int encoder,float set)
{
    static float PWM_out,Encoder_Err,Encoder_S,EnC_Err_Lowout_last;
    float a=0.4;

    Encoder_Err=set-encoder;

    EnC_Err_Lowout=(1-a)*Encoder_Err+a*EnC_Err_Lowout_last;
    EnC_Err_Lowout_last=EnC_Err_Lowout;//防止速度过大的影响直立环的正常工作。
    //3.对速度偏差积分，积分出位移
    Encoder_S+=EnC_Err_Lowout;//【4】
    //4.积分限幅
    Encoder_S=Encoder_S>12000?12000:(Encoder_S<(-12000)?(-12000):Encoder_S);
    //5.速度环控制输出计算
    if(Start_Flag == 0)
        Encoder_S = 0;

    PWM_out=X_Velocity_Kp*EnC_Err_Lowout+X_Velocity_Ki*Encoder_S;//【5】
    b = Encoder_S;
    c = X_Velocity_Ki*Encoder_S;

    return PWM_out;
}

/*********************
速度环PI：Kp*Ek+Ki*Ek_S
*********************/
float M_Velocity(int encoder,float set)
{
    static float PWM_out,Encoder_Err,Encoder_S,EnC_Err_Lowout,EnC_Err_Lowout_last;//【2】
    float a=0.7;//【3】

    //1.计算速度偏差
    Encoder_Err=set-encoder;//舍去误差
    //2.对速度偏差进行低通滤波
    //low_out=(1-a)*Ek+a*low_out_last;
    EnC_Err_Lowout=(1-a)*Encoder_Err+a*EnC_Err_Lowout_last;//使得波形更加平滑，滤除高频干扰，防止速度突变。
    EnC_Err_Lowout_last=EnC_Err_Lowout;//防止速度过大的影响直立环的正常工作。
    //3.对速度偏差积分，积分出位移
    Encoder_S+=EnC_Err_Lowout;//【4】
    //4.积分限幅
    Encoder_S=Encoder_S>1000?1000:(Encoder_S<(-1000)?(-1000):Encoder_S);
    //5.速度环控制输出计算
    if(Start_Flag == 0)
        Encoder_S = 0;

    PWM_out=M_Velocity_Kp*EnC_Err_Lowout+M_Velocity_Ki*Encoder_S;//【5】
    return PWM_out;
}

/*********************
速度环PI：Kp*Ek+Ki*Ek_S
*********************/
float T_Velocity(int encoder,float set)
{
    static float PWM_out,Encoder_Err,Encoder_S,EnC_Err_Lowout,EnC_Err_Lowout_last;//【2】
    float a=0.7;//【3】

    //1.计算速度偏差
    Encoder_Err=set-encoder;//舍去误差
    //2.对速度偏差进行低通滤波
    //low_out=(1-a)*Ek+a*low_out_last;
    EnC_Err_Lowout=(1-a)*Encoder_Err+a*EnC_Err_Lowout_last;//使得波形更加平滑，滤除高频干扰，防止速度突变。
    EnC_Err_Lowout_last=EnC_Err_Lowout;//防止速度过大的影响直立环的正常工作。
    //3.对速度偏差积分，积分出位移
    Encoder_S+=EnC_Err_Lowout;//【4】
    //4.积分限幅
    Encoder_S=Encoder_S>10000?10000:(Encoder_S<(-10000)?(-10000):Encoder_S);
    //5.速度环控制输出计算
    if(Start_Flag == 0)
        Encoder_S = 0;

    PWM_out=T_Velocity_Kp*EnC_Err_Lowout+T_Velocity_Ki*Encoder_S;//【5】
    return PWM_out;
}
