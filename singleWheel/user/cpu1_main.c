/*********************************************************************************************************************
* TC264 Opensourec Library 即（TC264 开源库）是一个基于官方 SDK 接口的第三方开源库
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 TC264 开源库的一部分
*
* TC264 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有隐含的适销性或适合特定用途的保证
* 更多细节请参见 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请参阅<https://www.gnu.org/licenses/>
*
* 额外注明：
* 本开源库使用 GPL3.0 开源许可证协议 以上许可申明为译文版本
* 许可申明英文版在 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件中
* 许可证副本在 libraries 文件夹下 即该文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序 但修改内容时必须保留逐飞科技的版权声明（即本声明）
*
* 文件名称          cpu1_main
* 公司名称          成都逐飞科技有限公司
* 版本信息          查看 libraries/doc 文件夹内 version 文件 版本说明
* 开发环境          ADS v1.9.4
* 适用平台          TC264D
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2022-09-15       pudding            first version
********************************************************************************************************************/

#include "head.h"
#pragma section all "cpu1_dsram"
// 将本语句与#pragma section all restore语句之间的全局变量都放在CPU1的RAM中

// **************************** 代码区域 ****************************

// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设
// 本例程是开源库空工程 可用作移植或者测试各类内外设

const char* tmp[17] = {"B0 ", "B1 ", "B2 ", "B3 ", "B4 ", "B5 ", "B6 ","B7 ", "B8 ", "B9", "B10", "B11", "B12", "B13", "B14", "B15", "B16"};
// #$%& 下 上 左 右

uint16 lit_idx = 0;

void core1_main(void)
{
    disable_Watchdog();                     // 关闭看门狗
    interrupt_global_enable(0);             // 打开全局中断
    // 此处编写用户代码 例如外设初始化代码等
    dot_matrix_screen_init();               // 点阵屏幕初始化
    cpu_wait_event_ready();                 // 等待所有核心初始化完毕
//    int lasthasFound = 0;
    while (TRUE)
    {
        if (image_flag) {
            SCC8660_Task();
        }
        else {
            image_offset = 0;
            to_image_offset = 0;
        }
//        if (lasthasFound && lasthasFound != hasFoundTargetBoard) fall_flag = 1;
//        lasthasFound = hasFoundTargetBoard;
        dataprocessing();
        dot_matrix_screen_set_brightness(3000);
//
//        // 切换显示Bx
        if (lit_idx < sp_point_index - 1 && light_points[lit_idx] == navigation.cur_index) {
            show_index = lit_idx + 1;
            lit_idx += 1;
        }

        // 显示
        if (show_index) {
            dot_matrix_screen_show_string(tmp[show_index]);
        }
        else {
            if (Start_Flag == 0) {
                if (angle_offset > 20) dot_matrix_screen_show_string("NG&");
                else if (angle_offset < -20) dot_matrix_screen_show_string("NG%");
                else dot_matrix_screen_show_string("NG$");
            }
            else {
                if (angle_offset > 20) dot_matrix_screen_show_string("OK&");
                else if (angle_offset < -20) dot_matrix_screen_show_string("OK%");
                else dot_matrix_screen_show_string("OK$");
            }
        }
    }
}
#pragma section all restore
// **************************** 代码区域 ****************************
