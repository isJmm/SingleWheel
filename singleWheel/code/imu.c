#include "imu.h"

#define cheat

#define M_delta_T 0.002f // 2ms计算一次
#define alpha 0.3f
#define M_PI 3.1415926f

float GyroOffset_Xdata = 0, AccOffset_Xdata = 0, icm_data_acc_x = 0, icm_data_gyro_x = 0;
float GyroOffset_Ydata = 0, AccOffset_Ydata = 0, icm_data_acc_y = 0, icm_data_gyro_y = 0;
float GyroOffset_Zdata = 0, AccOffset_Zdata = 0, icm_data_acc_z = 0, icm_data_gyro_z = 0;
float Q_info_q0 = 1, Q_info_q1 = 0, Q_info_q2 = 0, Q_info_q3 = 0;

float start_angle = 0;

float param_Kp = 1.5;  // 加速度计的收敛速率比例增益
float param_Ki = 0.004; // 陀螺仪收敛速率的积分增益 0.004

float M_eulerAngle_yaw = 0, M_eulerAngle_pitch = 0, M_eulerAngle_roll = 0;
float M_eulerAngle_yaw_old = 0, M_eulerAngle_yaw_total = 0;

float M_I_ex, M_I_ey, M_I_ez; // 误差积分

MadgwickAHRS mdw = {
        .sampleFreq = 500.0f,
        .betaDef = 0.1f,
        .beta = 0.5f,
        .q0 = 1.0f,
        .q1 = 0.0f,
        .q2 = 0.0f,
        .q3 = 0.0f,
};

MahonyAHRS mhy = {
        .sampleFreq = 500.0f,
        .twoKpDef = (2.0f * 0.5f),
        .twoKiDef = (2.0f * 0.0f),
        .twoKp = (2.0f * 1.5f),
        .twoKi = (2.0f * 0.004f),
        .q0 = 1.0f,
        .q1 = 0.0f,
        .q2 = 0.0f,
        .q3 = 0.0f,
        .integralFBx = 0.0f,
        .integralFBy = 0.0f,
        .integralFBz = 0.0f,
};


//陀螺仪零飘初始化
void gyroOffset_init(void) {
    GyroOffset_Xdata = 0;
    GyroOffset_Ydata = 0;
    GyroOffset_Zdata = 0;
    for (uint16_t i = 0; i < 1000; i++) {
        imu660ra_get_gyro();
        GyroOffset_Xdata += imu660ra_gyro_x;
        GyroOffset_Ydata += imu660ra_gyro_y;
        GyroOffset_Zdata += imu660ra_gyro_z;
        system_delay_ms(5);
    }
     GyroOffset_Xdata /= 1000;
     GyroOffset_Ydata /= 1000;
     GyroOffset_Zdata /= 1000;
}

void imu_init(void) {
    imu660ra_init();
    gyroOffset_init();
}

// 转化为实际物理值
void ICM_getValues() {
    // 一阶低通滤波，单位g/s
    icm_data_acc_x = (((float)imu660ra_acc_x / 4096.) * alpha) + icm_data_acc_x * (1 - alpha);
    icm_data_acc_y = (((float)imu660ra_acc_y / 4096.) * alpha) + icm_data_acc_y * (1 - alpha);
    icm_data_acc_z = (((float)imu660ra_acc_z / 4096.) * alpha) + icm_data_acc_z * (1 - alpha);
    // 陀螺仪角速度转弧度
    icm_data_gyro_x = ((float)imu660ra_gyro_x - GyroOffset_Xdata) * M_PI / 180 / 16.4f;
    icm_data_gyro_y = ((float)imu660ra_gyro_y - GyroOffset_Ydata) * M_PI / 180 / 16.4f;
#ifdef cheat
    if (fabs(imu660ra_gyro_z - GyroOffset_Zdata) < 0.003f) {
        icm_data_gyro_z = 0;
    }
    else {
        icm_data_gyro_z = ((float)imu660ra_gyro_z - GyroOffset_Zdata) * M_PI / 180 / 16.4f;
    }
#else
    icm_data_gyro_z = ((float)imu660ra_gyro_z - GyroOffset_Zdata) * M_PI / 180 / 16.4f;
#endif
}

// 互补滤波
void ICM_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az) {
    float halfT = 0.5 * M_delta_T;
    float vx, vy, vz; // 当前的机体坐标系上的重力单位向量
    float ex, ey, ez; // 四元数计算值与加速度计测量值的误差
    float q0 = Q_info_q0;
    float q1 = Q_info_q1;
    float q2 = Q_info_q2;
    float q3 = Q_info_q3;
    float q0q0 = q0 * q0;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    //float q0q3 = q0 * q3;
    float q1q1 = q1 * q1;
    //float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;
    // float delta_2 = 0;

    // 对加速度数据进行归一化 得到单位加速度
    float norm = invSqrt(ax * ax + ay * ay + az * az);

    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    // 根据当前四元数的姿态值来估算出各重力分量。用于和加速计实际测量出来的各重力分量进行对比，从而实现对四轴姿态的修正
    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;
    // vz = (q0*q0-0.5f+q3 * q3) * 2;

    // 叉积来计算估算的重力和实际测量的重力这两个重力向量之间的误差。
    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    // 用叉乘误差来做PI修正陀螺零偏，
    // 通过调节 param_Kp，param_Ki 两个参数，
    // 可以控制加速度计修正陀螺仪积分姿态的速度。
    M_I_ex += halfT * ex; // integral error scaled by Ki
    M_I_ey += halfT * ey;
    M_I_ez += halfT * ez;

    gx = gx + param_Kp * ex + param_Ki * M_I_ex;
    gy = gy + param_Kp * ey + param_Ki * M_I_ey;
    gz = gz + param_Kp * ez + param_Ki * M_I_ez;

    /*数据修正完成，下面是四元数微分方程*/

    // 四元数微分方程，其中halfT为测量周期的1/2，gx gy gz为陀螺仪角速度，以下都是已知量，这里使用了一阶龙哥库塔求解四元数微分方程
    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
    q1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
    q2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
    q3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;
    //    delta_2=(2*halfT*gx)*(2*halfT*gx)+(2*halfT*gy)*(2*halfT*gy)+(2*halfT*gz)*(2*halfT*gz);
    //    整合四元数率    四元数微分方程  四元数更新算法，二阶毕卡法
    //    q0 = (1-delta_2/8)*q0 + (-q1*gx - q2*gy - q3*gz)*halfT;
    //    q1 = (1-delta_2/8)*q1 + (q0*gx + q2*gz - q3*gy)*halfT;
    //    q2 = (1-delta_2/8)*q2 + (q0*gy - q1*gz + q3*gx)*halfT;
    //    q3 = (1-delta_2/8)*q3 + (q0*gz + q1*gy - q2*gx)*halfT

    // normalise quaternion
    norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    Q_info_q0 = q0 * norm;
    Q_info_q1 = q1 * norm;
    Q_info_q2 = q2 * norm;
    Q_info_q3 = q3 * norm;

}

// MadgwickAHRS算法
void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-mdw.q1 * gx - mdw.q2 * gy - mdw.q3 * gz);
    qDot2 = 0.5f * (mdw.q0 * gx + mdw.q2 * gz - mdw.q3 * gy);
    qDot3 = 0.5f * (mdw.q0 * gy - mdw.q1 * gz + mdw.q3 * gx);
    qDot4 = 0.5f * (mdw.q0 * gz + mdw.q1 * gy - mdw.q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * mdw.q0;
        _2q1 = 2.0f * mdw.q1;
        _2q2 = 2.0f * mdw.q2;
        _2q3 = 2.0f * mdw.q3;
        _4q0 = 4.0f * mdw.q0;
        _4q1 = 4.0f * mdw.q1;
        _4q2 = 4.0f * mdw.q2;
        _8q1 = 8.0f * mdw.q1;
        _8q2 = 8.0f * mdw.q2;
        q0q0 = mdw.q0 * mdw.q0;
        q1q1 = mdw.q1 * mdw.q1;
        q2q2 = mdw.q2 * mdw.q2;
        q3q3 = mdw.q3 * mdw.q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * mdw.q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * mdw.q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * mdw.q3 - _2q1 * ax + 4.0f * q2q2 * mdw.q3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= mdw.beta * s0;
        qDot2 -= mdw.beta * s1;
        qDot3 -= mdw.beta * s2;
        qDot4 -= mdw.beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    mdw.q0 += qDot1 * (1.0f / mdw.sampleFreq);
    mdw.q1 += qDot2 * (1.0f / mdw.sampleFreq);
    mdw.q2 += qDot3 * (1.0f / mdw.sampleFreq);
    mdw.q3 += qDot4 * (1.0f / mdw.sampleFreq);

    // Normalise quaternion
    recipNorm = invSqrt(mdw.q0 * mdw.q0 + mdw.q1 * mdw.q1 + mdw.q2 * mdw.q2 + mdw.q3 * mdw.q3);
    mdw.q0 *= recipNorm;
    mdw.q1 *= recipNorm;
    mdw.q2 *= recipNorm;
    mdw.q3 *= recipNorm;
}

// MahonyAHRS 算法
void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
    float recipNorm;
    float halfvx, halfvy, halfvz;
    float halfex, halfey, halfez;
    float qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity and vector perpendicular to magnetic flux
        halfvx = mhy.q1 * mhy.q3 - mhy.q0 * mhy.q2;
        halfvy = mhy.q0 * mhy.q1 + mhy.q2 * mhy.q3;
        halfvz = mhy.q0 * mhy.q0 - 0.5f + mhy.q3 * mhy.q3;

        // Error is sum of cross product between estimated and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if(mhy.twoKi > 0.0f) {
            mhy.integralFBx += mhy.twoKi * halfex * (1.0f / mhy.sampleFreq);    // integral error scaled by Ki
            mhy.integralFBy += mhy.twoKi * halfey * (1.0f / mhy.sampleFreq);
            mhy.integralFBz += mhy.twoKi * halfez * (1.0f / mhy.sampleFreq);
            gx += mhy.integralFBx;  // apply integral feedback
            gy += mhy.integralFBy;
            gz += mhy.integralFBz;
        }
        else {
            mhy.integralFBx = 0.0f; // prevent integral windup
            mhy.integralFBy = 0.0f;
            mhy.integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += mhy.twoKp * halfex;
        gy += mhy.twoKp * halfey;
        gz += mhy.twoKp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5f * (1.0f / mhy.sampleFreq));     // pre-multiply common factors
    gy *= (0.5f * (1.0f / mhy.sampleFreq));
    gz *= (0.5f * (1.0f / mhy.sampleFreq));
    qa = mhy.q0;
    qb = mhy.q1;
    qc = mhy.q2;
    mhy.q0 += (-qb * gx - qc * gy - mhy.q3 * gz);
    mhy.q1 += (qa * gx + qc * gz - mhy.q3 * gy);
    mhy.q2 += (qa * gy - qb * gz + mhy.q3 * gx);
    mhy.q3 += (qa * gz + qb * gy - qc * gx);

    // Normalise quaternion
    recipNorm = invSqrt(mhy.q0 * mhy.q0 + mhy.q1 * mhy.q1 + mhy.q2 * mhy.q2 + mhy.q3 * mhy.q3);
    mhy.q0 *= recipNorm;
    mhy.q1 *= recipNorm;
    mhy.q2 *= recipNorm;
    mhy.q3 *= recipNorm;
}

// 获取车辆姿态
void ICM_getEulerianAngles(void) {
    // 采集陀螺仪数据
    imu660ra_get_gyro();
    imu660ra_get_acc();
    // 转化为实际物理量
    ICM_getValues();
    // 姿态解算
    ICM_AHRSupdate(icm_data_gyro_x, icm_data_gyro_y, icm_data_gyro_z, icm_data_acc_x, icm_data_acc_y, icm_data_acc_z);
//    MadgwickAHRSupdateIMU(icm_data_gyro_x, icm_data_gyro_y, icm_data_gyro_z, icm_data_acc_x, icm_data_acc_y, icm_data_acc_z);
//    MahonyAHRSupdateIMU(icm_data_gyro_x, icm_data_gyro_y, icm_data_gyro_z, icm_data_acc_x, icm_data_acc_y, icm_data_acc_z);
    float q0 = Q_info_q0;
    float q1 = Q_info_q1;
    float q2 = Q_info_q2;
    float q3 = Q_info_q3;

    // 四元数计算欧拉角---原始
    M_eulerAngle_roll = asin(-2 * q1 * q3 + 2 * q0 * q2) * 180 / M_PI;                                // pitch
    M_eulerAngle_pitch = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 180 / M_PI; // roll
    M_eulerAngle_yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180 / M_PI;  // yaw

    if (fabs(M_eulerAngle_yaw) < 0.1) M_eulerAngle_yaw = 0;

    float delta = M_eulerAngle_yaw - M_eulerAngle_yaw_old;
    if (delta < -180)     delta += 360;
    else if (delta > 180) delta -= 360;

    M_eulerAngle_yaw_total += delta;
    M_eulerAngle_yaw_old = M_eulerAngle_yaw;
}




