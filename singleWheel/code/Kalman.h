// Kalman.h 的 C 语言实现

#ifndef KALMAN_H
#define KALMAN_H

// 定义 Kalman 结构体，包含滤波器所需的所有参数和状态
typedef struct {
    float Q_angle;     // 角度的过程噪声协方差
    float Q_bias;      // 偏置的过程噪声协方差
    float R_measure;   // 测量噪声协方差
    float angle;       // 估计的角度
    float bias;        // 估计的偏置
    float rate;        // 未经滤波的测量速率
    float P[2][2];     // 误差协方差矩阵
} Kalman;

// 初始化 Kalman 结构体
void Kalman_Init(Kalman* kalman);

// 获取经过滤波的角度
float Kalman_getAngle(Kalman* kalman, float newAngle, float newRate, float dt);

// 设置滤波器估计的角度
void Kalman_setAngle(Kalman* kalman, float angle);

// 获取当前速率
float Kalman_getRate(Kalman* kalman);

// 设置过程噪声协方差 Q_angle
void Kalman_setQangle(Kalman* kalman, float Q_angle);

// 设置过程噪声协方差 Q_bias
void Kalman_setQbias(Kalman* kalman, float Q_bias);

// 设置测量噪声协方差 R_measure
void Kalman_setRmeasure(Kalman* kalman, float R_measure);

// 获取过程噪声协方差 Q_angle
float Kalman_getQangle(Kalman* kalman);

// 获取过程噪声协方差 Q_bias
float Kalman_getQbias(Kalman* kalman);

// 获取测量噪声协方差 R_measure
float Kalman_getRmeasure(Kalman* kalman);

#endif
