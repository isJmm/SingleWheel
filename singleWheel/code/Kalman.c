// Kalman.c 的 C 语言实现

#include "Kalman.h"

// 初始化 Kalman 滤波器的参数
void Kalman_Init(Kalman* kalman) {
    kalman->Q_angle = 0.001f;  // 初始化角度过程噪声
    kalman->Q_bias = 0.003f;   // 初始化偏置过程噪声
    kalman->R_measure = 0.03f; // 初始化测量噪声

    kalman->angle = 0.0f;      // 初始化估计角度
    kalman->bias = 0.0f;       // 初始化估计偏置

    // 初始化误差协方差矩阵为零
    kalman->P[0][0] = 0.0f;
    kalman->P[0][1] = 0.0f;
    kalman->P[1][0] = 0.0f;
    kalman->P[1][1] = 0.0f;
}

// 使用卡尔曼滤波器计算新的估计角度
float Kalman_getAngle(Kalman* kalman, float newAngle, float newRate, float dt) {
    // 1. 预测
    kalman->rate = newRate - kalman->bias;   // 减去偏置后的真实速率
    kalman->angle += dt * kalman->rate;      // 预测新的角度

    // 更新误差协方差矩阵
    kalman->P[0][0] += dt * (dt*kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0] + kalman->Q_angle);
    kalman->P[0][1] -= dt * kalman->P[1][1];
    kalman->P[1][0] -= dt * kalman->P[1][1];
    kalman->P[1][1] += kalman->Q_bias * dt;

    // 2. 更新
    float S = kalman->P[0][0] + kalman->R_measure; // 计算测量协方差
    float K[2];
    K[0] = kalman->P[0][0] / S;  // 计算卡尔曼增益
    K[1] = kalman->P[1][0] / S;

    float y = newAngle - kalman->angle;  // 计算角度残差
    kalman->angle += K[0] * y;           // 更新角度
    kalman->bias += K[1] * y;            // 更新偏置

    // 更新误差协方差矩阵
    float P00_temp = kalman->P[0][0];
    float P01_temp = kalman->P[0][1];

    kalman->P[0][0] -= K[0] * P00_temp;
    kalman->P[0][1] -= K[0] * P01_temp;
    kalman->P[1][0] -= K[1] * P00_temp;
    kalman->P[1][1] -= K[1] * P01_temp;

    return kalman->angle; // 返回更新后的角度
}

// 设置滤波器估计的角度
void Kalman_setAngle(Kalman* kalman, float angle) { kalman->angle = angle; }

// 获取当前速率
float Kalman_getRate(Kalman* kalman) { return kalman->rate; }

// 设置过程噪声协方差 Q_angle
void Kalman_setQangle(Kalman* kalman, float Q_angle) { kalman->Q_angle = Q_angle; }

// 设置过程噪声协方差 Q_bias
void Kalman_setQbias(Kalman* kalman, float Q_bias) { kalman->Q_bias = Q_bias; }

// 设置测量噪声协方差 R_measure
void Kalman_setRmeasure(Kalman* kalman, float R_measure) { kalman->R_measure = R_measure; }

// 获取过程噪声协方差 Q_angle
float Kalman_getQangle(Kalman* kalman) { return kalman->Q_angle; }

// 获取过程噪声协方差 Q_bias
float Kalman_getQbias(Kalman* kalman) { return kalman->Q_bias; }

// 获取测量噪声协方差 R_measure
float Kalman_getRmeasure(Kalman* kalman) { return kalman->R_measure; }
