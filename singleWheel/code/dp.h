#ifndef _DP_H
#define _DP_H

#include "head.h"

typedef struct
{
    float yaokong_angle_offset;  //Ò£¿ØÆ«²î
    float image_offset;          //ÉãÏñÍ·Æ«²î
    float navigation_offset;     //¹ßµ¼Æ«²î
    float RTK_offset;            //¶¨Î»Æ«²î
}my_offset;

extern float speed;
extern float angle_offset;
extern float Dynamic_zero_set;
extern float yaokong_speed,yaokong_angle;
extern float K_zero_dot;
extern int slow_flag;
extern int fangxiang_flag;
extern float zero_set;//¶¯Ì¬Áãµã
extern float zero_Error;
extern float speed_test;
extern float angleUsed;
extern uint16 show_index;
extern float K1, K2;

void dataprocessing(void);
void anlgle_offset_handle(float yaokong_offset,float image_offset,float navigation_offset,float RTK_offset);
void speed_offset_handle(float speed_in);
float Dynamic_zero_cale(void);
void yaokong_getvalue(void);
void yaokong_check(void);
void slow_check(void);
void fangxiang_check(void);

#endif
