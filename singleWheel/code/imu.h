#ifndef CODE_IMU_H_
#define CODE_IMU_H_

#include "head.h"

typedef struct {
    float sampleFreq;      // sample frequency in Hz
    float betaDef;        // 2 * proportional gain
    volatile float beta;                              // 2 * proportional gain (Kp)
    volatile float q0, q1, q2, q3;  // quaternion of sensor frame relative to auxiliary frame
}MadgwickAHRS;

typedef struct {
    float sampleFreq;          // sample frequency in Hz
    float twoKpDef;   // 2 * proportional gain
    float twoKiDef;   // 2 * integral gain

    volatile float twoKp;                                            // 2 * proportional gain (Kp)
    volatile float twoKi;                                            // 2 * integral gain (Ki)
    volatile float q0, q1, q2, q3;                  // quaternion of sensor frame relative to auxiliary frame
    volatile float integralFBx,  integralFBy, integralFBz; // integral error terms scaled by Ki
}MahonyAHRS;

extern float GyroOffset_Xdata;
extern float GyroOffset_Ydata;
extern float GyroOffset_Zdata;
extern float GyroOffset_Xdata, icm_data_acc_x, icm_data_gyro_x;
extern float GyroOffset_Ydata, icm_data_acc_y, icm_data_gyro_y;
extern float GyroOffset_Zdata, icm_data_acc_z, icm_data_gyro_z;
extern float M_eulerAngle_yaw, M_eulerAngle_roll, M_eulerAngle_pitch;
extern float M_eulerAngle_yaw_old, M_eulerAngle_yaw_total;
extern float start_angle;
extern float Q_info_q0, Q_info_q1, Q_info_q2, Q_info_q3;

void imu_init(void);
void ICM_getEulerianAngles(void);
void ICM_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az);

#endif /* CODE_IMU_H_ */
