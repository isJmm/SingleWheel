#ifndef CODE_QUATER_QUATER_H_
#define CODE_QUATER_QUATER_H_

#include "head.h"
//===================================================宏定义===================================================
#define delta_T  0.005f
#define GYRO_SPL 16.4
#define ACC_SPL  4096.0
//===================================================宏定义===================================================

//欧拉角结构体
typedef struct
{
    double pitch;
    double roll;
    double yaw;
}eulerAngle_info_struct;

//四元数结构体
typedef struct {
    double q0;
    double q1;
    double q2;
    double q3;
}quater_info_struct;

//角速度漂移量
typedef struct
{
    double Xdata;
    double Ydata;
    double Zdata;
} gyroOffset_info_struct;

//加速度计速度漂移量
typedef struct
{
    double Xdata;
    double Ydata;
    double Zdata;
} accOffset_info_struct;

//===================================================变量声明===================================================
extern eulerAngle_info_struct eulerAngle;
extern gyroOffset_info_struct gyroOffset;
extern accOffset_info_struct accOffset;
extern double imu_kp;                                             //加速度计的收敛速率比例增益
extern double imu_ki;                                             //陀螺仪收敛速率的积分增益
extern double I_ex, I_ey, I_ez;                                  // 误差积分
extern double ex, ey, ez;
extern double gyro_z;
extern double bias;
//===================================================变量声明===================================================

//===================================================函数声明===================================================
void imu_task(void);
void Update_Angle(void);
void imu_data_deal(void);
void gyroOffsetInit(void);
void accOffsetInit(void);
void biasCorrect(void);
//===================================================函数声明===================================================
#endif /* CODE_QUATER_QUATER_H_ */
