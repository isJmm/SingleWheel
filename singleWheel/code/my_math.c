/*
 * math.c
 *
 *  Created on:
 *      Author:
 */

#include "my_math.h"

 extKalman_t Zero_Kalman;

int clip(int x, int low, int up) {
    return x > up ? up : x < low ? low : x;
}

float fclip(float x, float low, float up) {
    return x > up ? up : x < low ? low : x;
}

float invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

//快速计算 Sqrt(x)
float my_sqrt(float number)
{
    return invSqrt(number) * number;
}

int min(int para1, int para2) {
    if (para1 > para2) {
        return para2;
    }
    return para1;
}

int max(int para1, int para2) {
    if (para1 > para2) {
        return para1;
    }
    return para2;
}

uint16 umax(uint16 para1, uint16 para2) {
    if (para1 > para2) {
        return para1;
    }
    return para2;
}
uint16 umin(uint16 para1, uint16 para2) {
    if (para1 > para2) {
        return para2;
    }
    return para1;
}
float min_float(float para1, float para2) {
    if (para1 > para2) {
        return para2;
    }
    return para1;
}

float max_float(float para1, float para2) {
    if (para1 > para2) {
        return para1;
    }
    return para2;
}

float Gaussian_Function(int x, float avr, float sigma) {
    float t = -0.5 * (1.0 * x - avr / sigma) * (1.0 * x - avr / sigma);
    float res = 1.0 / (sigma * sqrt(2 * PI)) * exp(t);
    return res;
}

float Slope_Function(int x, float k, float intercept) {
    float res = 0;
    if (x >= 0) {
        res = -k * x + intercept;
    }
    else {
        res = k * x + intercept;
    }
    return max_float(res, 0);
}

float Map(float val,float in_min,float in_max,float out_min,float out_max)
{
    return (float)(val-in_min)*(out_max-out_min)/(in_max-in_min)+out_min;
}

float get_distance(float x1, float y1, float x2, float y2) {
    return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}


/**
  * @author  Liu heng
  * 一阶卡尔曼滤波器来自RoboMaster论坛
  *   一维卡尔曼滤波器
  *   使用时先定义一个kalman指针，然后调用kalmanCreate()创建一个滤波器
  *   每次读取到传感器数据后即可调用KalmanFilter()来对数据进行滤波
  *          使用示例
  *          extKalman p;                  //定义一个卡尔曼滤波器结构体
  *          float SersorData;             //需要进行滤波的数据
  *          KalmanCreate(&p,20,200);      //初始化该滤波器的Q=20 R=200参数
  *          while(1)
  *          {
  *             SersorData = sersor();                     //获取数据
  *             SersorData = KalmanFilter(&p,SersorData);  //对数据进行滤波
  *          }
  */

/**
  * @name   kalmanCreate
  * @brief  创建一个卡尔曼滤波器
  * @param  p:  滤波器
  *         T_Q:系统噪声协方差
  *         T_R:测量噪声协方差
  *
  * @retval none
  * @attention R固定，Q越大，代表越信任侧量值，Q无穷代表只用测量值
  *             反之，Q越小代表越信任模型预测值，Q为零则是只用模型预测
  */
void KalmanCreate(extKalman_t *p,float T_Q,float T_R)
{
    p->X_last = (float)0;
    p->P_last = 0;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1;
    p->B = 0;
    p->H = 1;
    p->X_mid = p->X_last;
}

/**
  * @name   KalmanFilter
  * @brief  卡尔曼滤波器
  * @param  p:  滤波器
  *         dat:待滤波数据
  * @retval 滤波后的数据
  * @attention Z(k)是系统输入,即测量值   X(k|k)是卡尔曼滤波后的值,即最终输出
  *            A=1 B=0 H=1 I=1  W(K)  V(k)是高斯白噪声,叠加在测量值上了,可以不用管
  *            以下是卡尔曼的5个核心公式
  *            一阶H'即为它本身,否则为转置矩阵
  */

float KalmanFilter(extKalman_t* p,float dat)
{
    p->X_mid  = p->A * p->X_last;                     //百度对应公式(1)    x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)
    p->P_mid  = p->A * p->P_last + p->Q;               //百度对应公式(2)    p(k|k-1) = A*p(k-1|k-1)*A'+Q
    p->kg     = p->P_mid / (p->P_mid + p->R);             //百度对应公式(4)    kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    p->X_now  = p->X_mid + p->kg * (dat - p->X_mid);     //百度对应公式(3)    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    p->P_now  = (1 - p->kg) * p->P_mid;                //百度对应公式(5)    p(k|k) = (I-kg(k)*H)*P(k|k-1)
    p->P_last = p->P_now;                         //状态更新
    p->X_last = p->X_now;
    return p->X_now;                              //输出预测结果x(k|k)
}




