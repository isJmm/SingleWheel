/*
 * ADIS16505-2.h
 *
 *  Created on: 2024ƒÍ1‘¬5»’
 *      Author: ljk
 */
#include "head.h"

#ifndef LIBRARIES_DOC_ADIS16505_2_H_
#define LIBRARIES_DOC_ADIS16505_2_H_

extern uint8 data_ready;
extern uint8 adis_init;

typedef struct ADIS16505_raw_data
{
        uint16 gy_z_high;//Z÷·ÃÿÕ”¬›“«∏ﬂ16Œª LSB
        uint16 gy_z_low;
        uint16 gy_x_high;
        uint16 gy_x_low;
        uint16 gy_y_high;
        uint16 gy_y_low;

        uint16 acc_x_high;
        uint16 acc_x_low;
        uint16 acc_y_high;
        uint16 acc_y_low;
        uint16 acc_z_high;
        uint16 acc_z_low;

        int16 temp;

} ADIS16505_raw_data;

typedef struct ADIS16505_true_data
{
        int32 gy_z;
        int32 gy_x;
        int32 gy_y;

        int32 acc_x;
        int32 acc_y;
        int32 acc_z;
        int16 temp;
} ADIS16505_true_data;

typedef struct ADIS16505_data
{
        float gx;
        float gy;
        float gz;
        float ax;
        float ay;
        float az;
        float temp;
} ADIS16505_data;

#define filter_set_high          0xDC06
#define filter_set_low           0xDD00   //¬À≤®∆˜B=6
#define filter_read              0x5C00
#define GYZ_LOW                  0x0C00  //Z÷·Õ”¬›“«µÕ16Œª
#define GYZ_OUT                  0x0E00  //Z÷·Õ”¬›“«∏ﬂ16Œª
#define GYX_LOW                  0x0400
#define GYX_OUT                  0x0600
#define GYY_LOW                  0x0800
#define GYY_OUT                  0x0A00
#define ACC_X_LOW                0x1000
#define ACC_X_HIGH               0x1200
#define ACC_Y_LOW                0x1400
#define ACC_Y_HIGH               0x1600
#define ACC_Z_LOW                0x1800
#define ACC_Z_HIGH               0x1A00
#define DEC_RATE_HIGH            (0xE403)
#define DEC_RATE_LOW             (0xE500)
#define TEMP                     0x1C00
#define RST1                     0xE880
#define RST2                     0xE900
#define Kg                       0.025
#define ACC_Kg                   0.00245
#define ADIS16505_SPI                         (SPI_1           )
#define ADIS16505_CS_PIN                      (P10_5          )
#define ADIS16505_RST_PIN                     (P20_10          )
#define ADIS16505_MOSI                        (SPI1_MOSI_P10_3)
#define ADIS16505_MISO                        (SPI1_MISO_P10_1)
#define ADIS16505_SCK                         (SPI1_SCLK_P10_2)
#define ADIS16505_SPEED                       (2 * 1000 * 1000 )

extern void ADIS16505_init(void);
extern void ADIS16505_Sampling_Callback(void);
void ADIS16505_task(void);
void get_gyroOffset(void);
void transfer_to_eulerAngle(void);
void adis16505_debug(void);
void ADIS16505_get_data(void);


extern ADIS16505_raw_data ADIS16505;
extern ADIS16505_true_data ADIS16505_true;
extern ADIS16505_data ADIS16505_;
#endif /* LIBRARIES_DOC_ADIS16505_2_H_ */
