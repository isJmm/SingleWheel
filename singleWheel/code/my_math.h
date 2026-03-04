/*
 * math.h
 *
 *  Created on: 2021年4月3日
 *      Author: 后
 */

#ifndef CODE_MY_MATH_H_
#define CODE_MY_MATH_H_

#include "head.h"

//快速计算 Sqrt(x)
float my_sqrt(float number);
int clip(int x, int low, int up);
float fclip(float x, float low, float up);
int min(int para1, int para2);
int max(int para1, int para2);
float max_float(float para1, float para2);
float min_float(float para1, float para2);
uint16 umax(uint16 para1, uint16 para2);
float Slope_Function(int x, float k, float intercept);
float Gaussian_Function(int x, float avr, float sigma);
float Map(float val,float in_min,float in_max,float out_min,float out_max);
uint16 umin(uint16 para1, uint16 para2);
uint16 umax(uint16 para1, uint16 para2);
float invSqrt(float x);

typedef struct {
    float X_last; //上一时刻的最优结果  X(k-|k-1)
    float X_mid;  //当前时刻的预测结果  X(k|k-1)
    float X_now;  //当前时刻的最优结果  X(k|k)
    float P_mid;  //当前时刻预测结果的协方差  P(k|k-1)
    float P_now;  //当前时刻最优结果的协方差  P(k|k)
    float P_last; //上一时刻最优结果的协方差  P(k-1|k-1)
    float kg;     //kalman增益
    float A;      //系统参数
    float B;
    float Q;
    float R;
    float H;
}extKalman_t;

//卡尔曼滤波
void KalmanCreate(extKalman_t *p,float T_Q,float T_R);
float KalmanFilter(extKalman_t* p,float dat);
float get_distance(float x1, float y1, float x2, float y2);
extern extKalman_t Zero_Kalman;

#endif /* CODE_MY_MATH_H_ */
