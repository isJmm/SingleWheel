#include "head.h"
#pragma section all "cpu0_dsram"

int core0_main(void)
{
    clock_init();                                                    // 获取时钟频率<务必保留>
    debug_init();                                                    // 初始化默认调试串口

    Init();
//    system_delay_ms(1000);
//    start_angle = M_eulerAngle_yaw;
    cpu_wait_event_ready(); // 等待所有核心初始化完毕

//    while (1) {
//        SCC8660_Task();
//        Para_show(HSV);
//    }

    Menu_Switch();

    while (TRUE)
    {
        /*
         * encoder_bdc
         * 加上 cheat 去除低通滤波试试
         * menu加上了err_scape
         * param_kp = 1.7
         * */

        /*
         * start_angle
         *
         * 先验证很远的点是不是能准确地到达
         *
         * 单独验证大拐弯是不是拐弯半径加上误差距离导致的不准
         *
         * 试一下z角速度积分
         * */
    }
}


#pragma section all restore
