
#ifndef _image_h_
#define _image_h_

#include "head.h"

#define IMAGE_H 30
#define IMAGE_W 40
#define MIN_BLOB_AREA 30 // 白点的开启阈值
#define MIN_CONE_AREA 50 // 锥桶颜色点数的开启阈值

#define COLOR_H_RANGE       10  // 设定色块标定的颜色范围
#define COLOR_S_RANGE       20  // 设定色块标定的对比度范围
#define COLOR_V_RANGE       20  // 设定色块标定的亮度范围

typedef struct {
        uint16 first;  // 第一个通道 R / H
        uint16 second; // 第二个通道 G / S
        uint16 third;  // 第三个通道 B / V
} color_channels;

typedef struct {
        uint16 first_max; // 第一个通道 R / H
        uint16 first_min;
        uint16 second_max;// 第二个通道 G / S
        uint16 second_min;
        uint16 third_max;// 第三个通道 B / V
        uint16 third_min;
} color_limits;

// 目标相应的信息
typedef struct {
        uint16 color;                   // 对象的颜色
        color_limits hsv_limits;        // hsv通道的限制
        color_limits rgb565_limits;     // rgb565三个通道的限制
} Objects;

// 二值化的图像
extern uint16 Threshold_Image[IMAGE_H][IMAGE_W];
//压缩的图像
extern uint16 Image_Used[IMAGE_H][IMAGE_W];

// 目标板相关信息
extern Objects targetboard;
// 分离图像的模式
enum IMAGE_MODE { RGB, HSV };

extern float image_offset;
extern float to_image_offset;
extern int hasFoundTargetBoard;
extern uint8 resetFlag;
extern uint8 fall_flag;
extern float image_flag;

// 函数声明
void SCC8660_Task(void);
void GetRgb565(uint16 data, color_channels* rgb);
void Rgb565ToHsv(color_channels* rgb, color_channels* hsv);
void Rgb565ToHsl(color_channels* rgb, color_channels* hsl);
color_channels HslToRgb565(color_channels* hsl);
void Para_show(enum IMAGE_MODE image_mode);
float findCentreOffset(uint16 image[][IMAGE_W]);

#define setMidColorToTarget() setColorToTarget(Image_Used[IMAGE_H / 2][IMAGE_W / 2], &targetboard.hsv_limits)

#endif

