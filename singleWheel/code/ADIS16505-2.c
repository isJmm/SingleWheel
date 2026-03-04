/*
 * ADIS16505-2.c
 *
 *  Created on: 2024年1月5日
 *      Author: ljk
 */

#include "ADIS16505-2.h"

ADIS16505_raw_data ADIS16505;
ADIS16505_true_data ADIS16505_true;
ADIS16505_data ADIS16505_;
uint8 adis_init = 0;

#define PI                 (3.1415926535898)

float gyroOffset_x = 0, gyroOffset_y = 0, gyroOffset_z = 0;

uint8 data_ready;

static uint16 SPI_Transfer(uint16 data_out)      //16位传输  内部调用
{
    uint16 resp;
    gpio_low(ADIS16505_CS_PIN );
    spi_transfer_16bit(ADIS16505_SPI,&data_out,&resp,1);
    gpio_high(ADIS16505_CS_PIN);
    return resp;
}

static void ADIS16505_Reset(void)
{
    SPI_Transfer(RST1);
    system_delay_us(20);
    SPI_Transfer(RST1);
    system_delay_us(20);
    SPI_Transfer(RST2);
}

void ADIS16505_init(void)
{
    adis_init = 1;
    spi_init(ADIS16505_SPI, SPI_MODE3, ADIS16505_SPEED, ADIS16505_SCK, ADIS16505_MOSI, ADIS16505_MISO , SPI_CS_NULL);
    gpio_init(ADIS16505_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    ADIS16505_Reset();
    system_delay_ms(20);
    SPI_Transfer(filter_set_high);   //设置内部滤波器N为6
    system_delay_ms(20);
    SPI_Transfer(filter_set_low);
    system_delay_ms(400);
    SPI_Transfer(0x7200);            //设备识别码
    system_delay_us(20);
    uint16 id = SPI_Transfer(0x7000);    //出厂时间
    system_delay_us(20);
    // printf("%X\n",id);
   if(id==0x4079)
   {
       // printf("ADIS16505-2 detected\n");
   }else
   {
       tft180_show_string(0, 0, "no device");
       // printf("No ADIS16505-2 detected\n");
       while(1);
   }
   SPI_Transfer(0x6E00);    // 年
   system_delay_us(20);
   SPI_Transfer(0x6C00);    // 月
   system_delay_us(20);
   SPI_Transfer(GYZ_OUT);   // 版本
   //printf("Time of manufacture:%x,%x\n",year,month_day);//芯片出厂年月日
   //printf("Firmware version:%x.0%x\n",version>>8,(uint8)version);//芯片固件版本
}

void ADIS16505_get_data(void) {
    ADIS16505.gy_z_high = SPI_Transfer(GYZ_LOW);    //与ADIS16505通信获取数据
    system_delay_us(20);
    ADIS16505.gy_z_low  = SPI_Transfer(GYX_OUT);
    system_delay_us(20);
    ADIS16505.gy_x_high = SPI_Transfer(GYX_LOW);
    system_delay_us(20);
    ADIS16505.gy_x_low  = SPI_Transfer(GYY_OUT);
    system_delay_us(20);
    ADIS16505.gy_y_high = SPI_Transfer(GYY_LOW);
    system_delay_us(20);
    ADIS16505.gy_y_low  = SPI_Transfer(ACC_X_HIGH);
    system_delay_us(20);
    ADIS16505.acc_x_high = SPI_Transfer(ACC_X_LOW);
    system_delay_us(20);
    ADIS16505.acc_x_low  = SPI_Transfer(ACC_Y_HIGH);
    system_delay_us(20);
    ADIS16505.acc_y_high =  SPI_Transfer(ACC_Y_LOW);
    system_delay_us(20);
    ADIS16505.acc_y_low  = SPI_Transfer(ACC_Z_HIGH);
    system_delay_us(20);
    ADIS16505.acc_z_high = SPI_Transfer(ACC_Z_LOW);
    system_delay_us(20);
    ADIS16505.acc_z_low  = SPI_Transfer(TEMP);
    system_delay_us(20);
    ADIS16505.temp = SPI_Transfer(GYZ_OUT);
    system_delay_us(20);
    ADIS16505_true.gy_x = ((uint32)ADIS16505.gy_x_high << 16) | ADIS16505.gy_x_low; //合并为32位数据
    ADIS16505_true.gy_y = ((uint32)ADIS16505.gy_y_high << 16) | ADIS16505.gy_y_low;
    ADIS16505_true.gy_z = ((uint32)ADIS16505.gy_z_high << 16) | ADIS16505.gy_z_low;
    ADIS16505_true.acc_x = ((uint32)ADIS16505.acc_x_high << 16) | ADIS16505.acc_x_low;
    ADIS16505_true.acc_y = ((uint32)ADIS16505.acc_y_high << 16) | ADIS16505.acc_y_low;
    ADIS16505_true.acc_z = ((uint32)ADIS16505.acc_z_high << 16) | ADIS16505.acc_z_low;
    ADIS16505_true.temp = ADIS16505.temp;
}

void get_gyroOffset(void) {
    for (int i = 0; i < 2000; i ++) {
        ADIS16505_get_data();
        gyroOffset_x += ADIS16505_true.gy_x;
        gyroOffset_y += ADIS16505_true.gy_y;
        gyroOffset_z += ADIS16505_true.gy_z;
        system_delay_ms(1);
    }
    gyroOffset_x /= 2000;
    gyroOffset_y /= 2000;
    gyroOffset_z /= 2000;
}


#define acc_alpha  0.3f
#define gyro_alpha 0.8f
//#define cheat

void get_real_value(void) {
    ADIS16505_.gx = ANGLE_TO_RAD((ADIS16505_true.gy_x - gyroOffset_x) * Kg / 65536) * gyro_alpha + ADIS16505_.gx * (1 - gyro_alpha);//转换为具有物理意义的数据
    ADIS16505_.gy = ANGLE_TO_RAD((ADIS16505_true.gy_y - gyroOffset_y) * Kg / 65536) * gyro_alpha + ADIS16505_.gy * (1 - gyro_alpha);

#ifdef cheat
    if (fabs(ADIS16505_true.gy_z - gyroOffset_z) < 0.1) ADIS16505_.gz = 0;
    else ADIS16505_.gz = ANGLE_TO_RAD((ADIS16505_true.gy_z - gyroOffset_z) * Kg / 65536) * gyro_alpha + ADIS16505_.gz * (1 - gyro_alpha);
#else
    ADIS16505_.gz = ANGLE_TO_RAD((ADIS16505_true.gy_z - gyroOffset_z) * Kg / 65536) * gyro_alpha + ADIS16505_.gz * (1 - gyro_alpha);
#endif

    ADIS16505_.ax = (((float)ADIS16505_true.acc_x * ACC_Kg / 65536) * acc_alpha) + (ADIS16505_.ax * (1 - acc_alpha));
    ADIS16505_.ay = (((float)ADIS16505_true.acc_y * ACC_Kg / 65536) * acc_alpha) + (ADIS16505_.ay * (1 - acc_alpha));
    ADIS16505_.az = (((float)ADIS16505_true.acc_z * ACC_Kg / 65536) * acc_alpha) + (ADIS16505_.az * (1 - acc_alpha));
    ADIS16505_.temp = ADIS16505_true.temp * 0.1;
}

void ADIS16505_Sampling_Callback(void)
{
    ADIS16505_get_data();
    get_real_value();
}

void transfer_to_eulerAngle(void) {
    float q0 = Q_info_q0;
    float q1 = Q_info_q1;
    float q2 = Q_info_q2;
    float q3 = Q_info_q3 ;

    M_eulerAngle_pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 180 / PI;
    M_eulerAngle_roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 180 / PI;
    M_eulerAngle_yaw = -(atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180 / PI) - start_angle;

    if (fabs(M_eulerAngle_yaw) < 0.1) M_eulerAngle_yaw = 0;

    float delta = M_eulerAngle_yaw - M_eulerAngle_yaw_old;
    if (delta < -180)     delta += 360;
    else if (delta > 180) delta -= 360;

    M_eulerAngle_yaw_total += delta;
    M_eulerAngle_yaw_old = M_eulerAngle_yaw;
}

void adis16505_debug(void) {
    while (1) {
        if(data_ready)
        {
            printf("%f %f %f %f %f %f\n",ADIS16505_.ax,ADIS16505_.ay,ADIS16505_.az,
                   ADIS16505_.gx,ADIS16505_.gy,ADIS16505_.gz);
            data_ready = 0;//输出x轴 y轴 z轴 加速度 角速率
        }
    }
}

void ADIS16505_task(void) {
    if (!adis_init) return;
    ADIS16505_get_data();
    get_real_value();
    ICM_AHRSupdate(ADIS16505_.gx, ADIS16505_.gy, ADIS16505_.gz, ADIS16505_.ax, ADIS16505_.ay, ADIS16505_.az);
    transfer_to_eulerAngle();
    data_ready = 1;
}


