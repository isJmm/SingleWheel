#include "quater.h"

double q0 = 1, q1 = 0, q2 = 0, q3 = 0;    // 初始位置姿态角为：0、0、0，对应四元数为：1、0、0、0
eulerAngle_info_struct eulerAngle;                            //欧拉角
gyroOffset_info_struct gyroOffset;
accOffset_info_struct accOffset;

double accoffsetx=120,accoffsety=-170,accoffsetz=-18;
double I_ex, I_ey, I_ez;                                  // 误差积分
//1ms
double imu_kp= 1.5;   //0.17                                              //加速度计的收敛速率比例增益
double imu_ki= 0.0005;        //0.004                                        //陀螺仪收敛速率的积分增益
double gyro_z;
double bias = 0.1;
/******************************变量定义************************************/

//陀螺仪解算姿态任务函数
void imu_task(void)
{
    // 数据获取
//    acc_get_angle();
//    gyro_get_angle();
    // 数据处理
    imu_data_deal();
    // 解算角度
    Update_Angle();
}

/*
 * @brief 计算陀螺仪零漂
 * 通过采集一定数据求均值计算陀螺仪零点偏移值。
 * 后可以用 陀螺仪读取的数据 - 零飘值，即可去除零点偏移量。
 */
void gyroOffsetInit(void)
{
    gyroOffset.Xdata = 0;
    gyroOffset.Ydata = 0;
    gyroOffset.Zdata = 0;

    for (uint16_t i = 0; i < 200; ++i)
    {
        imu660ra_get_gyro();
        gyroOffset.Xdata += imu660ra_gyro_x;
        gyroOffset.Ydata += imu660ra_gyro_y;
        gyroOffset.Zdata += imu660ra_gyro_z;

        system_delay_ms(5);
    }

    gyroOffset.Xdata /= 200;
    gyroOffset.Ydata /= 200;
    gyroOffset.Zdata /= 200;
}
void accOffsetInit(void)
{
    accOffset.Xdata = 0;
    accOffset.Ydata = 0;
    accOffset.Zdata = 0;

    for (uint16_t i = 0; i < 200; ++i)
    {
       imu660ra_get_acc();
       accOffset.Xdata += imu660ra_acc_x;
       accOffset.Ydata += imu660ra_acc_y;
       accOffset.Zdata += imu660ra_acc_z;

       system_delay_ms(5);
    }

      accOffset.Xdata /= 200;
      accOffset.Ydata /= 200;
      accOffset.Zdata /= 200;
}

// 递归的低通滤波器
/*
 * 2π×0.01 = 0.079577471546
 */
float Filter_gyro_z(float input)
{
    static float state;
    state = state + 0.002 / (0.079577471546 + 0.002) * (input - state);
    return state;
}

//imu数据处理
void imu_data_deal(void)
{
    float alpha = 0.2;  //0.35
    //一阶低通滤波，单位g
    imu.acc.acc[X] =  (imu660ra_acc_x - accOffset.Xdata) / ACC_SPL * alpha  + imu.acc.acc[X] * (1 - alpha);
    imu.acc.acc[Y] =  (imu660ra_acc_y - accOffset.Ydata) / ACC_SPL * alpha  + imu.acc.acc[Y] * (1 - alpha);
    imu.acc.acc[Z] =  (imu660ra_acc_z - accOffset.Zdata) / ACC_SPL * alpha  + imu.acc.acc[Z] * (1 - alpha);

    imu.gyro.gyro[X] = ANGLE_TO_RAD((imu660ra_gyro_x - gyroOffset.Xdata) / GYRO_SPL);
    imu.gyro.gyro[Y] = ANGLE_TO_RAD((imu660ra_gyro_y - gyroOffset.Ydata) / GYRO_SPL);
    imu.gyro.gyro[Z] = ANGLE_TO_RAD((imu660ra_gyro_z - gyroOffset.Zdata) / GYRO_SPL);

    gyro_z = imu660ra_gyro_z / GYRO_SPL;
    gyro_z = Filter_gyro_z(gyro_z) + bias;// 手动减零漂(符号反)
    if (gyro_z < 0.1 && gyro_z > -0.1) gyro_z = 0;
}

/* 偏置修正 */
void biasCorrect(void) {
    double gyro_z_tmp = 0, sum_gyro_z = 0;
    for (int i = 0; i < 1000; i ++) {
        imu660ra_get_gyro();
        gyro_z_tmp = Filter_gyro_z(imu660ra_gyro_z / GYRO_SPL);
        sum_gyro_z += gyro_z_tmp;
        system_delay_ms(5);
    }
    bias = -sum_gyro_z / 1000;
}

/**
 * @brief 用互补滤波算法解算陀螺仪姿态(即利用加速度计修正陀螺仪的积分误差)
 * 加速度计对振动之类的噪声比较敏感，长期数据计算出的姿态可信；陀螺仪对振动噪声不敏感，短期数据可信，
 * 但长期使用积分误差严重(内部积分算法放大静态误差)。
 * 因此使用姿态互补滤波，短期相信陀螺仪，长期相信加速度计。
 */
double ex, ey, ez;     // 当前加速计测得的重力加速度在三轴上的分量与用当前姿态计算得来的重力在三轴上的分量的误差
void Update_Angle(void)
{
    double halfT = 0.5 * delta_T;    // 采样周期一半

    double vx, vy, vz;     // 当前姿态计算得来的重力在三轴上的分量

    double q0q0 = q0 * q0;//先相乘，方便后续计算
    double q0q1 = q0 * q1;
    double q0q2 = q0 * q2;
    //double q0q3 = q0 * q3;//未使用
    double q1q1 = q1 * q1;
    //double q1q2 = q1 * q2;//未使用
    double q1q3 = q1 * q3;
    double q2q2 = q2 * q2;
    double q2q3 = q2 * q3;
    double q3q3 = q3 * q3;

    // 正常静止状态为-g 反作用力。
    if(imu.acc.acc[X] * imu.acc.acc[Y] * imu.acc.acc[Z] == 0) // 加速度计处于自由落体状态时(此时g = 0)不进行姿态解算，因为会产生分母无穷大的情况
        return;

    // 对加速度数据进行归一化得到单位加速度
    //加速度计<测量>的重力加速度向量(机体坐标系)
    double norm = 1 / my_sqrt(imu.acc.acc[X] *imu.acc.acc[X] + imu.acc.acc[Y]*imu.acc.acc[Y] + imu.acc.acc[Z]* imu.acc.acc[Z]);
    imu.acc.acc[X] = imu.acc.acc[X] * norm;
    imu.acc.acc[Y] = imu.acc.acc[Y] * norm;
    imu.acc.acc[Z] = imu.acc.acc[Z] * norm;

    //陀螺仪积分<估计>重力向量(机体坐标系)
    // 机体坐标系下重力在三个轴上的分量
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    //将加速度计获得的重力向量归一化后的值与提取的姿态矩阵的重力向量叉乘获取姿态误差
    ex = imu.acc.acc[Y] * vz - imu.acc.acc[Z]* vy;
    ey = imu.acc.acc[Z] * vx - imu.acc.acc[X]* vz;
    ez = imu.acc.acc[X] * vy - imu.acc.acc[Y]* vx;

    //对误差进行积分
    I_ex += ex;
    I_ey += ey;
    I_ez += ez;

    //将误差输入PID控制器后与陀螺仪测得的角速度相加，修正角速度值
    imu.gyro.gyro[X] = imu.gyro.gyro[X] + imu_kp* ex + imu_ki* I_ex;
    imu.gyro.gyro[Y] = imu.gyro.gyro[Y] + imu_kp* ey + imu_ki* I_ey;
    imu.gyro.gyro[Z] = imu.gyro.gyro[Z] + imu_kp* ez + imu_ki* I_ez;

    // 一阶龙格库塔法求解四元数微分方程，其中halfT为测量周期的1/2
    q0 = q0 + (-q1 * imu.gyro.gyro[X] - q2 * imu.gyro.gyro[Y] - q3 * imu.gyro.gyro[Z]) * halfT;
    q1 = q1 + ( q0 * imu.gyro.gyro[X] + q2 * imu.gyro.gyro[Z] - q3 * imu.gyro.gyro[Y]) * halfT;
    q2 = q2 + ( q0 * imu.gyro.gyro[Y] - q1 * imu.gyro.gyro[Z] + q3 * imu.gyro.gyro[X]) * halfT;
    q3 = q3 + ( q0 * imu.gyro.gyro[Z] + q1 * imu.gyro.gyro[Y] - q2 * imu.gyro.gyro[X]) * halfT;

    // 单位化四元数在空间旋转时不会拉伸，仅有旋转角度，下面算法类似线性代数里的正交变换
    norm = 1/sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);

    q0 = q0 * norm;
    q1 = q1 * norm;
    q2 = q2 * norm;
    q3 = q3 * norm;

    eulerAngle.pitch  = RAD_TO_ANGLE(atan2(2 * q2 * q3 + 2* q0 * q1, q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3));//-180~180
    eulerAngle.roll   = RAD_TO_ANGLE(asin(2 * q0 * q2 - 2 * q1 * q3));//-90~90
//    eulerAngle.yaw    = RAD_TO_ANGLE(atan2(2 * q1 * q2 + 2 * q0 * q3,  q0 * q0 +q1 * q1 - q2 * q2 - q3 * q3));//0~360

    // 对 z 轴的角加速的进行一个积分就得到的角度，但是长期来说是不准确的
    eulerAngle.yaw += gyro_z * 0.005035;
    if(eulerAngle.yaw>180) eulerAngle.yaw=-180;  if(eulerAngle.yaw<-180) eulerAngle.yaw=180;
}

