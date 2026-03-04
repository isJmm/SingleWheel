#ifndef CODE_NAVIGATION_H_
#define CODE_NAVIGATION_H_

#include "head.h"

//===================================================宏定义===================================================
#define MAX_POSITION_COUNT               (50)                    // 最大的位置点位数量
#define CAR_MODE_COUNT                   (3)                     // 小车模式的数量
#define ERROR_POSITION_ACCEPT            (navigation.err_scape)  // 80 稳 60 快 70 中等
#define FLASH_POSITION_PAGE              (5)                     // 将位置信息写入flash的扇区

#define NAVIGATION_PER_MILES             (150)                   // 编码器增加 NAVIGATION_PER_MILES 对应 1m

#define START_POINT_X_MIN                (0)                     // 点位的x坐标最小值
#define START_POINT_X_MAX                (tft180_width_max)      // 点位的y坐标最大值

#define START_POINT_Y_MIN                (0)                     // 点位的x坐标最小值
#define START_POINT_Y_MAX                (tft180_height_max)     // 点位的y坐标最大值
//=========================================================================================================


//=================================================结构体定义===================================================
typedef struct{
      float x_cur;
      float y_cur;
      float yaw_cur;
      float distance;
} car_state;

typedef enum {
    INERTIAL_NAVIGATION,                            // 惯导
    REMOTE_CONTROL,                                 // 遥控
    CAMERA,                                         // 摄像头
} carMode;

typedef struct {
    float INS_set_yaw;                              // 惯性导航设定值
    float INS_set_yaw_update;                       // 惯性导航设定偏航斜坡更新值
    float INS_set_yaw_total;                        // 惯性导航设定偏航斜坡更新值
    float INS_set_yaw_old;                          //
    float INS_cur_distance;                         // 惯性导航当前距离
    float start_yaw;                                // 起始的偏航角
    float cur_yaw;                                  // 当前的偏航角
    float x_cur;                                    // 当前x坐标
    float y_cur;                                    // 当前y坐标
    float x_set;                                    // 设定x坐标
    float y_set;                                    // 设定y坐标
    float x_start;                                  // 起始x坐标
    float y_start;                                  // 起始y坐标

    uint8  back_home;                                // 回库的标志位
    uint8  reverse_flag;                             // 速度反转的标志位
    uint8  start_flag;                               // 惯导开启的标志位
    uint8  clear_flag;                               // 惯导误差清空标志位
    uint16 get_index;                                // 目前正在采集的点位下标
    uint16 pos_index;                                // 已经采集坐标的下标
    uint16 cur_index;                                // 当前正在前往的坐标的下标索引

    float bdc_speed_max;                             // 行进轮最大速度
    float bdc_speed;                                 // 行进轮当前速度
    int   speed_direction;                           // 记录速度方向 (-1 倒车 1 正着跑)
    float speed1, speed2, speed3, speed4;            // 每个阶段的速度变量


    float  err_scape;                                // 点位可接受误差
    float  back_car_flag;                            // 是否开启倒车模式的标志位
    float  offset;                                   // 惯性导航的偏差，用在转向环上面
} INS_struct;
//=========================================================================================================


//=================================================变量声明===================================================
// 灯光秀相关变量
extern int light_points[];
extern uint8 sp_flag;
extern int sp_point_index;

// 可视化图像的相关变量
extern uint16 region_length;
extern uint16 region_width;

// 小车模式相关变量
extern carMode car_mode[CAR_MODE_COUNT];             // 定义小车的状态
extern int car_mode_index;                           // 用于显示小车当前状态的辅助变量
extern carMode cur_car_mode;                         // 当前小车的模式
extern uint8 use_left_bottom;                        // 是否从左下角出发

// 惯性导航相关变量
extern car_state car_position[MAX_POSITION_COUNT];
extern INS_struct navigation;

// 编码器相关变量
extern float encoder_bdc_total;                      // 行进轮有刷电机脉冲计数的累计值
extern float encoder_bdc;                            // 行进轮有刷电机脉冲计数
extern float encoder_bdc_get;                        // 当前读取的编码器的值

// 欧拉角相关变量
extern float eulerAngle_yaw;                         // 解算出来的yaw角度
extern float eulerAngle_yaw_total;                   // yaw 角度的一个累计值
extern float eulerAngle_yaw_old;                     // 相对于eulerAngle_yaw的旧值
//=========================================================================================================


//=================================================函数声明===================================================
void INS_Update(void);
void INS_Init(void);
void Para_Update(void);
void GetPosInfo(void);
void Navigation_Handler(void);
void Go_home(void);
void navigation_para_show(void);
void navi_savePositionToFlash(void);
void navi_readPositionFromFlash(void);
void navi_clearAllPos(void);
void navigationStart(void);
void navigationReset(void);
void navigationTask(void);
void direction_recognition(void);
void encoder_get(void);
void encoder_get_10ms(void);
void eulerAngleUpdate(void);
void add_position(void);
void del_position(void);
void modify_position(void);
void query_position(void);
void auto_adapt_size(void);
void draw_digit(uint16 x, uint16 y, int digit);
//=========================================================================================================


#endif /* CODE_NAVIGATION_H_ */
