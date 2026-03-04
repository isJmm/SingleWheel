/*****************************************************************/
// /* Menu.c
//  *
//  *  Created on: 2023/3/7
//  *  Author: SHOCKY

/*****************************************************************/

//#include "zf_device_scc8660.h"
#include "Menu.h"

menu_item *current_menu_item;
float change_level = 1;
float angle_flag = 0;
float gyro_flag = 0;
float acc_flag = 0;
float encoder_flag = 0;
float image_yuanshi_flag = 0;
float image_erzhihua_flag = 0;
float row_line = 0;
float column_line = 0;
float sideline_flag = 0;
float image_state_flag = 0;
int change_mode = 1;

const char* carMode_string[] = {
        "NAVIGATION",
        "REMOTE",
        "CAMERA",
};

void State_Reverse(void);

void Save_Parameter(void)
{
    if(flash_check(0, 0))
        flash_erase_page(0, 0);
    flash_buffer_clear();
    flash_union_buffer[0].float_type  = M.Gyro.pid.P;
    flash_union_buffer[1].float_type  = M.Gyro.pid.I;
    flash_union_buffer[2].float_type  = M.Gyro.pid.D;
    flash_union_buffer[3].float_type  = M.Angle.pid.P;
    flash_union_buffer[4].float_type  = M_Velocity_Kp;

//    flash_union_buffer[5].float_type  = M_Velocity_Ki;
    flash_union_buffer[5].float_type  = navigation.back_car_flag;

    flash_union_buffer[6].float_type  = Xing.Gyro.pid.P;
    flash_union_buffer[7].float_type  = Xing.Gyro.pid.I;
    flash_union_buffer[8].float_type  = Xing.Angle.pid.P;
    flash_union_buffer[9].float_type  = Xing.Angle.pid.D;
    flash_union_buffer[10].float_type  = X_Velocity_Kp;

//    flash_union_buffer[11].float_type  = X_Velocity_Ki;
    flash_union_buffer[11].float_type  = exposure;

    flash_union_buffer[12].float_type  = T.Gyro.pid.P;
//    flash_union_buffer[13].float_type  = T.Gyro.pid.I;
    flash_union_buffer[14].float_type  = T.Angle.pid.P;
//    flash_union_buffer[15].float_type  = T.Angle.pid.D;
//    flash_union_buffer[16].float_type  = T_Velocity_Kp;
//    flash_union_buffer[17].float_type  = T_Velocity_Ki;
    flash_union_buffer[20].float_type  = navigation.err_scape;
    flash_union_buffer[19].float_type  = angle_flag;
//    flash_union_buffer[20].float_type  = gyro_flag;
    flash_union_buffer[21].float_type  = image_flag;
    flash_union_buffer[22].float_type  = encoder_flag;
//    flash_union_buffer[23].float_type  = image_yuanshi_flag;
//    flash_union_buffer[23].float_type  = K2;
//    flash_union_buffer[24].float_type  = K1;
//    flash_union_buffer[25].float_type  = K2;
    flash_union_buffer[26].float_type  = navigation.speed1;
    flash_union_buffer[27].float_type  = navigation.speed2;
    flash_union_buffer[28].float_type  = navigation.speed3;
    flash_union_buffer[29].float_type  = navigation.speed4;
//    flash_union_buffer[30].float_type  = k_offset;
    flash_union_buffer[31].float_type  = M_Med_Angle;
    flash_union_buffer[13].float_type  = Med_Angle;
    flash_union_buffer[33].float_type  = Start_Flag;
    flash_union_buffer[18].float_type  = K1;
    flash_union_buffer[35].float_type  = K2;
//    flash_union_buffer[36].float_type  = K1;
//    flash_union_buffer[37].float_type  = K2;

    if((flash_write_page_from_buffer(0, 0)) == 0)
    {
        gpio_low(P20_9);
        system_delay_ms(100);
        gpio_high(P20_9);
    }
}

void Read_Parameter(void)
{
    flash_buffer_clear();
    flash_read_page_to_buffer(0, 0);
    M.Gyro.pid.P = flash_union_buffer[0].float_type;
    M.Gyro.pid.I = flash_union_buffer[1].float_type;
    M.Gyro.pid.D= flash_union_buffer[2].float_type;
    M.Angle.pid.P = flash_union_buffer[3].float_type;
    M_Velocity_Kp = flash_union_buffer[4].float_type;

    // M_Velocity_Ki = flash_union_buffer[5].float_type;
    navigation.back_car_flag = flash_union_buffer[5].float_type;

    Xing.Gyro.pid.P = flash_union_buffer[6].float_type;
    Xing.Gyro.pid.I = flash_union_buffer[7].float_type;
    Xing.Angle.pid.P = flash_union_buffer[8].float_type;
    Xing.Angle.pid.D = flash_union_buffer[9].float_type;
    X_Velocity_Kp = flash_union_buffer[10].float_type;

//    X_Velocity_Ki = flash_union_buffer[11].float_type;
    exposure = flash_union_buffer[11].float_type;

    T.Gyro.pid.P = flash_union_buffer[12].float_type;
//    T.Gyro.pid.I = flash_union_buffer[13].float_type;
    T.Angle.pid.P = flash_union_buffer[14].float_type;
//    T.Angle.pid.D = flash_union_buffer[15].float_type;
//    T_Velocity_Kp = flash_union_buffer[16].float_type;
//    T_Velocity_Ki = flash_union_buffer[17].float_type;
    navigation.err_scape = flash_union_buffer[20].float_type;
    angle_flag = flash_union_buffer[19].float_type;
//    gyro_flag = flash_union_buffer[20].float_type;
    image_flag = flash_union_buffer[21].float_type;
    encoder_flag = flash_union_buffer[22].float_type;
//    image_yuanshi_flag = flash_union_buffer[23].float_type;
//    K2 = flash_union_buffer[23].float_type;
//    K1 = flash_union_buffer[24].float_type;
//    K2 = flash_union_buffer[25].float_type;
    navigation.speed1 = flash_union_buffer[26].float_type;
    navigation.speed2 = flash_union_buffer[27].float_type;
    navigation.speed3 = flash_union_buffer[28].float_type;
    navigation.speed4 = flash_union_buffer[29].float_type;
//    k_offset = flash_union_buffer[30].float_type;
    M_Med_Angle = flash_union_buffer[31].float_type;
    Med_Angle = flash_union_buffer[13].float_type;
    Start_Flag = flash_union_buffer[33].float_type;
    K1 = flash_union_buffer[18].float_type;
    K2 = flash_union_buffer[35].float_type;
}

void Clear_All_Parameter(void)
{
    if(flash_check(0, 0))
        flash_erase_page(0, 0);

    flash_buffer_clear();
    flash_union_buffer[0].float_type  = 0.0;
    flash_union_buffer[1].float_type  = 0.0;
    flash_union_buffer[2].float_type  = 0.0;
    flash_union_buffer[3].float_type  = 0.0;
    flash_union_buffer[4].float_type  = 0.0;
    flash_union_buffer[5].float_type  = 0.0;
    flash_union_buffer[6].float_type  = 0.0;
    flash_union_buffer[7].float_type  = 0.0;
    flash_union_buffer[8].float_type  = 0.0;
    flash_union_buffer[9].float_type  = 0.0;
    flash_union_buffer[10].float_type  = 0.0;
    flash_union_buffer[11].float_type  = 0.0;
    flash_union_buffer[12].float_type  = 0.0;
    flash_union_buffer[13].float_type  = 0.0;
    flash_union_buffer[14].float_type  = 0.0;
    flash_union_buffer[15].float_type  = 0.0;
    flash_union_buffer[16].float_type  = 0.0;
    flash_union_buffer[17].float_type  = 0.0;
    flash_union_buffer[18].float_type  = 0.0;
    flash_union_buffer[19].float_type  = 0.0;
    flash_union_buffer[20].float_type  = 0.0;
    flash_union_buffer[21].float_type  = 0.0;
    flash_union_buffer[22].float_type  = 0.0;
    flash_union_buffer[23].float_type  = 0.0;
    flash_union_buffer[24].float_type  = 0.0;
    flash_union_buffer[25].float_type  = 0.0;
    flash_union_buffer[26].float_type  = 0.0;
    flash_union_buffer[27].float_type  = 0.0;
    flash_union_buffer[28].float_type  = 0.0;
    flash_union_buffer[29].float_type  = 0.0;
    flash_union_buffer[30].float_type  = 0.0;
    flash_union_buffer[31].float_type  = 0.0;
    flash_union_buffer[32].float_type  = 0.0;
    flash_union_buffer[33].float_type  = 0.0;
    flash_union_buffer[34].float_type  = 0.0;
    flash_union_buffer[35].float_type  = 0.0;
    flash_union_buffer[36].float_type  = 0.0;
    flash_union_buffer[37].float_type  = 0.0;

    if((flash_write_page_from_buffer(0, 0)) == 0)
    {
        gpio_low(P20_9);
        system_delay_ms(100);
        gpio_high(P20_9);
    }
}

void color_update(void) {

}

//void set_brightness(void) {
//    if (scc8660_set_brightness(exposure)) {
//        tft180_clear();
//        tft180_show_string(0, 0, "err");
//        system_delay_ms(2000);
//    }
//    else {
//        tft180_clear();
//        tft180_show_string(0, 0, "ok");
//        system_delay_ms(2000);
//    }
//}

menu_item menu[] = {
//    {01, "IMAGE", NULL, NULL},                                             // 閸ユ儳鍎氬Ο鈥崇础
//    {11, "level", NULL, &change_level},
//    {12, "Image_yuanshi", State_Reverse, &image_yuanshi_flag},
//    {13, "Image_erzhihua", State_Reverse, &image_erzhihua_flag},
//    {14, "Sideline", State_Reverse, &sideline_flag},
//    {15, "row_line", NULL, &row_line},
//    {16, "column_line", NULL, &column_line},
//    {17, "image_state", State_Reverse, &image_state_flag},
//    {18, "Save", Save_Parameter, NULL},
//    {19, "Return", NULL, NULL},

     /*
      *  tips:编号不能在数字前面加一个0,否则表示的是八进制，那么08,09就是一个非法的数字，警告)
      *       如果是07,06...这些没什么影响
      */
    {01, "level", NULL, &change_level},

    // RTK
    {04, "navigation", NULL, NULL},
    {41, "start", navigationStart, NULL},
    {42, "getPos", GetPosInfo, NULL},
    {43, "save", navi_savePositionToFlash, NULL},
    {44, "reset", navigationReset, NULL},
    {45, "query", query_position, NULL},
    {46, "param", NULL, NULL},
    {460, "level", NULL, &change_level},
    {460, "speed1", NULL, &navigation.speed1},
    {461, "speed2", NULL, &navigation.speed2},
    {462, "speed3", NULL, &navigation.speed3},
    {463, "speed4", NULL, &navigation.speed4},
    {464, "err", NULL, &navigation.err_scape},

    // RTK

    {02, "PID", NULL, NULL},                                               // PID
    {21, "Balance", NULL, NULL},                                            // 楠炲疇
    {211, "level", NULL, &change_level},
    {212, "gyro_kp", NULL, &M.Gyro.pid.P},
    {213, "gyro_ki", NULL, &M.Gyro.pid.I},
    {214, "gyro_kd", NULL, &M.Gyro.pid.D},
    {215, "angle_kp", NULL, &M.Angle.pid.P},
//    {215, "angle_kd", NULL, &M.Angle.pid.D},
    {216, "speed_kp", NULL, &M_Velocity_Kp},
//    {217, "speed_ki", NULL, &M_Velocity_Ki},
    {218, "M_Med_Angle", NULL, &M_Med_Angle},
    {22, "Advance", NULL, NULL},                                           // 閸撳秷绻�
    {221, "level", NULL, &change_level},
    {222, "gyro_kp", NULL, &Xing.Gyro.pid.P},
    {223, "gyro_ki", NULL, &Xing.Gyro.pid.I},
    {224, "angle_kp", NULL, &Xing.Angle.pid.P},
    {225, "angle_kd", NULL, &Xing.Angle.pid.D},
    {226, "speed_kp", NULL, &X_Velocity_Kp},
//    {227, "speed_ki", NULL, &X_Velocity_Ki},
    {228, "X_Med_Angle", NULL, &Med_Angle},
    {23, "Turn", NULL, NULL},                                              // 鏉烆剙鎮�
    {231, "level", NULL, &change_level},
    {232, "gyro_kp", NULL, &T.Gyro.pid.P},
//    {233, "gyro_ki", NULL, &T.Gyro.pid.I},
    {233, "angle_kp", NULL, &T.Angle.pid.P},
//    {235, "angle_kd", NULL, &T.Angle.pid.D},
//    {236, "speed_kp", NULL, &T_Velocity_Kp},
//    {237, "speed_ki", NULL, &T_Velocity_Ki},
//    {238, "k_offset", NULL, &k_offset},

    {03, "STATE", NULL, NULL},                                             // 閸欐﹢鍣洪惄鎴炲付
    {31, "Angle", State_Reverse, &angle_flag},
    {34, "Encoder", State_Reverse, &encoder_flag},
    {35,"START_FLAG",State_Reverse,&Start_Flag},
    {36,"image_flag",State_Reverse, &image_flag},
    {37,"back_car",State_Reverse, &navigation.back_car_flag},
//    {38,"exposure",NULL, &exposure},
//    {38,"set", set_brightness, NULL},

//    {04, "K1", NULL, &K1},
    {05, "K1", NULL, &K1},
    {05, "K2", NULL, &K2},
    //{05, "CLEAR", Clear_All_Parameter, NULL}                          // 濞撳懏顨熼幍锟介張澶婂棘閺佸府绱欓棁锟界憰浣瑰瀵拷閿涘矂妲诲銏ｎ嚖閹垮秳缍�
};

void State_Reverse(void)
{
    for (int i = 0; i < sizeof(menu) / sizeof(menu[0]); i++)
    {
        if (menu[i].menu_id == current_menu_item->menu_id)
        {
            *current_menu_item->param = !(*current_menu_item->param);
        }
    }
}

void State_Display(void)
{
    char txt[30];

    if(angle_flag)
    {
        tft180_show_float(0,16*9,M_eulerAngle_roll,3,2);
        tft180_show_float(40,16*9,M_eulerAngle_pitch,3,2);
        tft180_show_float(80,16*9,M_eulerAngle_yaw,3,2);
    }

//    if(gyro_flag)
//    {
//        sprintf(txt,"gx:%-5d",gyro[0]);
//        tft180_show_string(80, 32, txt);
//        sprintf(txt,"gy:%-5d",gyro[1]);
//        tft180_show_string(80, 48, txt);
//        sprintf(txt,"gz:%-5d",gyro[2]);
//        tft180_show_string(80, 64, txt);
//    }

//    if(acc_flag)
//    {
//        sprintf(txt,"ax:%-6d",accel[0]);
//        tft180_show_string(72, 80, txt);
//        sprintf(txt,"ay:%-6d",accel[1]);
//        tft180_show_string(72, 96, txt);
//        sprintf(txt,"az:%-6d",accel[2]);
//        tft180_show_string(72, 112, txt);
//    }

    if(encoder_flag)
    {
        sprintf(txt,"Ea:%-4d",Encoder_A);
        tft180_show_string(0, 16*9, txt);
        sprintf(txt,"Eb:%-4d",Encoder_B);
        tft180_show_string(40, 16*9, txt);
        sprintf(txt,"Ec:%-4d",Encoder_C);
        tft180_show_string(80, 16*9, txt);
    }
}

bool have_sub_menu(int menu_id)
{
    for (int i = 0; i < sizeof(menu) / sizeof(menu[0]); i++)
    {
        if (menu[i].menu_id / 10 == menu_id)
        {
            return true;
        }
    }
    return false;
}

int show_sub_menu(menu_item menu[], int sz, int parent_id, int highlight_col)
{
    tft180_clear();
    int item_idx = 0;
    for (int i = 0; i < sz; i ++)
    {
        if (menu[i].menu_id / 10 == parent_id)
        {
            if (item_idx == highlight_col)
            {
                current_menu_item = &menu[i];
                tft180_set_color(RGB565_RED,RGB565_WHITE);
            }
            else
            {
                tft180_set_color(RGB565_BLACK, RGB565_WHITE);
            }
            tft180_show_string(1, 16*item_idx, menu[i].menu_name);

            if (!strcmp(menu[i].menu_name, "carMode")) {
                tft180_show_string(50, 16*item_idx, carMode_string[change_mode]);
            }
            else if(menu[i].param != NULL)
                tft180_show_float(50, 16*item_idx, *menu[i].param, 3, 6);

            item_idx++;
        }
    }
    tft180_set_color(RGB565_RED, RGB565_WHITE);

    return item_idx;
}

void printFunc(int paraCnt, ...) {
    if (!wireless_init_flag || !Start_Flag) return;
    static char txt[30] = {0};
    va_list args;            // 声明用于存储参数列表的变量
    va_start(args, args);    // 初始化参数列表

    float para;
    char end = ',';
    for (int i = 0; i < paraCnt; i ++) {
        memset(txt,0,20);
        if (i == paraCnt - 1) end = '\n';
        para = va_arg(args, float);
        sprintf(txt, " %.3f%c", para, end);
        wireless_uart_send_string(txt);
    }

    va_end(args);  // 结束参数列表的处理
}

char txt[30] = {0};
void Menu_Switch(void)
{
    tft180_set_font(TFT180_6X8_FONT);
    uint8 level_index = 5;
    int parent_menu_id = 0;
    int highlight_col = 0;
    int menu_item_count = show_sub_menu(menu, sizeof(menu) / sizeof(menu[0]), parent_menu_id, highlight_col);
    float level[8] = {0.000001, 0.00001, 0.0001, 0.001, 0.01, 0.1, 1, 10};
    uint8 imageShowFlag = 0;
    while (1)
    {
//        ADIS16505_get_data(); 直接卡死
#if COLLECT
        tft180_show_float(50, 120, angleUsed, 3, 1);
//        tft180_show_int(50, 100, speed_reversal_flag, 1);
//        tft180_show_float(50, 80, navigation.inertial_navigation_set_yaw_update, 3, 1);
//        tft180_show_float(50, 60, navigation.x_cur, 3, 1);
//        tft180_show_float(50, 100, navigation.y_cur, 3, 1);

//        tft180_show_int(50, 60, sp_point_index, 2);
//        memset(txt,0,20);
//        sprintf(txt, "%.3f\n", Dynamic_zero_set);
//        wireless_uart_send_string(txt);
        if (!wireless_init_flag) {
            if (lora3a22_state_flag == 1 && lora3a22_uart_transfer.key[0] == 0 && lora3a22_uart_transfer.key[1] == 0) {
//                bdc_speed_max = 150;
//                Go_home();
            }
            if (get_point_flag && cur_car_mode == REMOTE_CONTROL && lora3a22_state_flag == 1 && lora3a22_uart_transfer.key[0] == 0) {
                GetPosInfo();
            }
        }
#endif
        if (imageShowFlag) {
            // 显示原始图像
            tft180_show_rgb565_image(0, 80, Image_Used[0], IMAGE_W, IMAGE_H, IMAGE_W, IMAGE_H, 0);
            // 显示二值化图像
            tft180_show_rgb565_image(40, 80, Threshold_Image[0], IMAGE_W, IMAGE_H, IMAGE_W, IMAGE_H, 0);
        }

        if (key_switch())
        {
            if (key_state[0])
            {
                if (key_state[0] == KEY_SHORT_PRESS) {
                    highlight_col--;
                    if (highlight_col < 0) highlight_col = menu_item_count - 1;
                }
                else if (key_state[0] == KEY_LONG_PRESS) {
                    imageShowFlag = !imageShowFlag;
                    system_delay_ms(1000);
                }
            }
            else if (key_state[1] == KEY_SHORT_PRESS)
            {
                highlight_col++;
                if (highlight_col >= menu_item_count) highlight_col = 0;
            }
            else if (key_state[2])
            {
                if (key_state[2] == KEY_SHORT_PRESS) {
                    if (have_sub_menu(current_menu_item->menu_id))
                    {
                        highlight_col = 0;
                        parent_menu_id = current_menu_item->menu_id;
                    }
                    else if (current_menu_item->menu_action)
                    {
                        current_menu_item->menu_action();
                        Save_Parameter();
                    }
                }
                else if (key_state[2] == KEY_LONG_PRESS) {
                    if (!strcmp(current_menu_item->menu_name, "getPos") && key_state[2] == KEY_LONG_PRESS) {
                        if (navigation.get_index == 0) continue;
                        navigation.get_index -= 1;
                        navigation.pos_index = navigation.get_index;
                        Buzzer_on();
                        system_delay_ms(50);
                        Buzzer_off();
                        tft180_show_int(50, 50, navigation.get_index, 2);
                        system_delay_ms(500);
                        continue;
                    }

                    if((current_menu_item->menu_id)/10 != 0)
                    {
                        highlight_col = 0;
                        parent_menu_id = (current_menu_item->menu_id)/100;
                        menu_item_count = show_sub_menu(menu, sizeof(menu) / sizeof(menu[0]), parent_menu_id, highlight_col);
                        system_delay_ms(1000);
                        continue;
                    }
                }
            }
            else if (key_state[3])
            {
                if (key_state[3] == KEY_SHORT_PRESS) {
                    if((strcmp(current_menu_item->menu_name, "level") == 0))
                    {
                        level_index = (level_index + 1) % 8;
                        change_level = level[level_index];
                    }
                    else if(current_menu_item->param != NULL)
                    {
                        *current_menu_item->param += change_level;
                        Save_Parameter();
                    }
                }
                else if (key_state[3] == KEY_LONG_PRESS) {
                    if(current_menu_item->param != NULL)
                    {
                        while (key_switch() && key_state[3] == KEY_LONG_PRESS) {
                            *current_menu_item->param += change_level;
                            Save_Parameter();
                            show_sub_menu(menu, sizeof(menu) / sizeof(menu[0]), parent_menu_id, highlight_col);
                            system_delay_ms(50);
                        }
                    }
                }
            }
            else if (key_state[4])
            {
                if (key_state[4] == KEY_SHORT_PRESS) {
                    if((strcmp(current_menu_item->menu_name, "level") == 0))
                    {
                        level_index = (level_index + 7) % 8;
                        change_level = level[level_index];
                    }
                    else if(current_menu_item->param != NULL)
                    {
                        *current_menu_item->param -= change_level;
                        Save_Parameter();
                    }
                }
                else if (key_state[4] == KEY_LONG_PRESS) {
                    if(current_menu_item->param != NULL)
                    {
                        while (key_switch() && key_state[4] == KEY_LONG_PRESS) {
                            *current_menu_item->param -= change_level;
                            Save_Parameter();
                            show_sub_menu(menu, sizeof(menu) / sizeof(menu[0]), parent_menu_id, highlight_col);
                            system_delay_ms(50);
                        }
                    }

                }
            }
            else if (key_state[5]) {
                if (key_state[5] == KEY_SHORT_PRESS)  {
                    Start_Flag = !Start_Flag;
                    system_delay_ms(50);
                }
                else if (key_state[5] == KEY_LONG_PRESS) {
                    sp_flag = 1;
                    GetPosInfo();
                    sp_flag = 0;
                    system_delay_ms(1000);
                }
            }
            menu_item_count = show_sub_menu(menu, sizeof(menu) / sizeof(menu[0]), parent_menu_id, highlight_col);
            key_clear_all_state();
        }

        State_Display();
    }
}
