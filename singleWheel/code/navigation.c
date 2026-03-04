#include "navigation.h"

//#pragma section all "cpu1_dsram"

// 灯关秀相关变量
int light_points[30];                      // 存灯关秀得的点位坐标
uint8 sp_flag = 0;                         // 灯光秀特殊点位的标志位
int sp_point_index = 0;                    // 灯光秀点位下标索引
extern uint16 lit_idx;

// 可视化图像相关变量
uint8 use_left_bottom = 0;                 // 起点判断标志位
point whole_bias, tmp_bias;                // 坐标偏置
uint16 cur_select = 1;                     // 用于指定选中的点位
uint8 position_update = 0;                 // 根据点位是否有修改来判断是否要刷新图像
int operation_select = 0;                  // 用于指定选中的操作
uint16 region_length = 8;                  // 场地的长度 单位 m
uint16 region_width = 8;                   // 场地的宽度 单位 m

// 小车模式相关变量
carMode cur_car_mode;
int car_mode_index = 0;
carMode car_mode[CAR_MODE_COUNT] = {
        INERTIAL_NAVIGATION,                // 惯导
        REMOTE_CONTROL,                     // 遥控
        CAMERA,                             // 摄像头
};

// 惯性导航相关变量
INS_struct navigation = { 0, };             // 惯性导航的变量
car_state car_position[MAX_POSITION_COUNT]; // 惯性导航的点位坐标

// 编码器相关变量
float encoder_bdc_total = 0;                // 行进轮有刷电机脉冲计数的累计值
float encoder_bdc = 0;                      // 行进轮有刷电机脉冲计数
float encoder_bdc_get = 0;                  // 当前读取的编码器的值
float low_pass = 0.6;                       // 低通滤波的参数

// 欧拉角相关变量
float eulerAngle_yaw = 0;                   // 解算出来的yaw角度
float eulerAngle_yaw_total = 0;             // yaw角度的一个累计值
float eulerAngle_yaw_old = 0;               // 相对于eulerAngle_yaw的旧值

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     惯性导航初始化
// 参数说明     void
// 返回参数     void
// 使用示例     INS_Init();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void INS_Init(void) {
    //设置当前坐标系为基准坐标系
    navigation.start_yaw = eulerAngle_yaw_total;
    navigation.speed_direction = 1;
    navigation.err_scape = 50;
    navigation.x_cur   = 0;
    navigation.y_cur   = 0;
    navigation.x_start = 0;
    navigation.y_start = 0;
    navigation.x_set   = 0;
    navigation.y_set   = 0;

    navigation.speed1 = 120;
    navigation.speed2 = 160;
    navigation.speed3 = 240;
    navigation.speed4 = 300;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     用于采集小车指定位置的坐标
// 参数说明     index       要采集的点位的下标
// 返回参数     void
// 使用示例     get_pos(index);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void get_pos(uint16 index) {
    if (index >= navigation.pos_index) return; // 防止越界访问
    car_position[index].x_cur = navigation.x_cur;
    car_position[index].y_cur = navigation.y_cur;
    car_position[index].yaw_cur = navigation.cur_yaw;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     清空指定点位的坐标
// 参数说明     index       要清除的点位的下标
// 返回参数     void
// 使用示例     clear_car_pos(index);
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void clear_car_pos(uint16 index) {
    if (index >= navigation.pos_index) return; // 防止越界访问
    car_position[index].distance = 0;
    car_position[index].x_cur = 0;
    car_position[index].y_cur = 0;
    car_position[index].yaw_cur = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器采集
// 参数说明     void
// 返回参数     void
// 使用示例     encoder_get();
// 备注信息     放在 5ms 中断中
//-------------------------------------------------------------------------------------------------------------------
void encoder_get(void) {
    // 将错误的数据滤掉
    if (fabs(Encoder_C) > 500) {
        encoder_bdc_get = 0;
    }
    else encoder_bdc_get = Encoder_C;

    // 低通滤波
    encoder_bdc = encoder_bdc * low_pass + encoder_bdc_get * (1 - low_pass);
    encoder_bdc_total += -encoder_bdc;  // 正负号要看实际情况而定，保证encoder_bdc_total是正数即可
    encoder_bdc_get = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     编码器采集
// 参数说明     void
// 返回参数     void
// 使用示例     encoder_get_10ms();
// 备注信息     放在 5ms 中断中, 冗余函数，没什么用
//-------------------------------------------------------------------------------------------------------------------
void encoder_get_10ms(void)
{
    static uint8 time_cnt = 0;
    time_cnt ++;
    if (time_cnt < 2) {
        encoder_bdc_get += Encoder_C;
    }
    else {
       time_cnt = 0;
       // 低通滤波
       encoder_bdc = encoder_bdc * low_pass + encoder_bdc_get * (1 - low_pass);
       encoder_bdc_total += -encoder_bdc;
       encoder_bdc_get = 0;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     惯性导航任务
// 参数说明     void
// 返回参数     void
// 使用示例     navigationTask();
// 备注信息     放在 2ms 中断中，只需要调用这一个函数即可
//-------------------------------------------------------------------------------------------------------------------
void navigationTask(void) {
    // yaw角更新
    eulerAngleUpdate();
    // 坐标系刷新
    INS_Update();
    // 方向判断
    if (navigation.back_car_flag)
        direction_recognition();
    // 参数更新
    Para_Update();
    // 点位切换
    Navigation_Handler();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     增加惯性导航点位
// 参数说明     void
// 返回参数     void
// 使用示例     add_position();
// 备注信息     在选中的点(cur_select)之前加一个点位
//-------------------------------------------------------------------------------------------------------------------
void add_position(void) {
    tft180_clear();
    uint16 add_pos = cur_select;
    tft180_clear();
    // 将坐标整体往后移
    for (uint16 i = navigation.pos_index; i > add_pos; i --) {
        car_position[i] = car_position [i - 1];
    }
    // 更新灯光秀的下标
    for (int i = 0; i < sp_point_index; i ++ ) {
        if (light_points[i] >= add_pos) {
            light_points[i] += 1;
        }
    }
    tft180_show_string(0, 0, "input the position!");
    system_delay_ms(500);

    while (TRUE) {
        if (key_state[2] == KEY_SHORT_PRESS){
            get_pos(add_pos);
            position_update = 1;
            navigation.pos_index ++; // 可能会溢出设定的值, 没有做保护
            navi_savePositionToFlash();
            tft180_clear();
            tft180_show_string(0, 0, "add success");
            system_delay_ms(500);
            return;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     删除惯性导航点位
// 参数说明     void
// 返回参数     void
// 使用示例     del_position();
// 备注信息     删除所选中的点位
//-------------------------------------------------------------------------------------------------------------------
void del_position(void) {
    tft180_clear();
    uint16 del_pos = cur_select;
    tft180_clear();
    // 整体向前移动一格
    for (uint16 i = del_pos; i < navigation.pos_index - 1; i ++) {
        car_position[i] = car_position [i + 1];
    }
    // 清空最后一个坐标
    clear_car_pos(navigation.pos_index - 1);
    position_update = 1;
    navigation.pos_index --; // 可能会溢出设定的值
    navi_savePositionToFlash();
    tft180_clear();
    tft180_show_string(0, 0, "del success");
    system_delay_ms(500);

    // 更新灯光秀的下标
    for (int i = 0; i < sp_point_index; i ++) {
        if (light_points[i] == del_pos) {
            for (int j = i; j < sp_point_index - 1; j ++) {
                light_points[j] = light_points[j + 1] - 1;
            }
            return;
        }
        else if (light_points[i] > del_pos) {
            light_points[i] -= 1;
        }
    }
    return;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新惯性导航点位
// 参数说明     void
// 返回参数     void
// 使用示例     modify_position();
// 备注信息     更新选中的点(cur_select)
//-------------------------------------------------------------------------------------------------------------------
void modify_position(void) {
    tft180_clear();
    uint16 modify_pos = cur_select;
    tft180_show_string(0, 0, "input the position!");
    system_delay_ms(500);
    while (TRUE) {
        if (key_state[2] == KEY_SHORT_PRESS){
            // 直接对相应的坐标进行修改即可
            get_pos(modify_pos);
            position_update = 1;
            navi_savePositionToFlash();
            tft180_clear();
            tft180_show_string(0, 0, "modify success");
            system_delay_ms(500);
            return;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     菜单取消操作
// 参数说明     void
// 返回参数     void
// 使用示例     cancel();
// 备注信息     取消菜单的当前操作，并且刷新点位
//-------------------------------------------------------------------------------------------------------------------
void cancel(void) {
    position_update = 1;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     自动更新场地
// 参数说明     void
// 返回参数     void
// 使用示例     auto_adapt_size();
// 备注信息     根据踩点的位置自动更新场地的大小，在 position_show()中调用
//-------------------------------------------------------------------------------------------------------------------
void auto_adapt_size(void) {
    static const uint16 inf = INT16_MAX - 1;
    point pt = {inf, inf};
    float left = inf, right = -inf, up = inf, down = -inf;
    // 获取已经采集的点位中边界值
    for (int i = 0; i < navigation.pos_index; i ++ ) {
        left = min_float(left, car_position[i].y_cur);
        right = max_float(right, car_position[i].y_cur);
        up = min_float(up, car_position[i].x_cur);
        down = max_float(down, car_position[i].x_cur);
    }

    // 计算所需要的长度和宽度
    float length = fabs(right - left), width = fabs(down - up);

    // 根据索要显示的位置计算所需要平移的距离
    if (start_pos == RIGHT_BOTTOM) {
        pt.x = down, pt.y = -min_float(left, 0);
    }
    else if (start_pos == LEFT_BOTTOM) {
        pt.x = -min_float(up, 0), pt.y = -min_float(left, 0);
    }

    // 将所计算的长度和宽度映射到实际的长度和宽度 单位m
    region_length = (length + NAVIGATION_PER_MILES - 1) / NAVIGATION_PER_MILES + 1;
    region_width  = (width + NAVIGATION_PER_MILES - 1) / NAVIGATION_PER_MILES + 1;

    // 计算将所有的点位容纳进屏幕中需要进行的整体平移坐标 (tft180坐标系中的坐标)
    whole_bias.x = pt.x / (region_width * NAVIGATION_PER_MILES) * tft180_width_max + 5;
    whole_bias.y = pt.y / (region_length * NAVIGATION_PER_MILES) * tft180_height_max + 5;
}

// 在指定的位置(x, y)写相应的数字(digit)

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     在坐标的点位绘制数字
// 参数说明     x        屏幕坐标系中的 x
// 参数说明     y        屏幕坐标系中的 y
// 参数说明     digit    所需要绘制的数字
// 返回参数     void
// 使用示例     draw_digit(50, 50, 1);
// 备注信息     这个函数有一定的bug，偶尔会出现越界的情况
//-------------------------------------------------------------------------------------------------------------------
void draw_digit(uint16 x, uint16 y, int digit) {
    tft180_set_font(TFT180_6X8_FONT);
    tft180_set_color(RGB565_BLACK, RGB565_WHITE);
    if (y < tft180_height_max - 18) {
        if (x + 10 < tft180_width_max - 18) {
            tft180_show_int(x + 10, y, digit , 2);
        }
        else {
            tft180_show_int(x - 10, y, digit , 2);
        }
    }
    else {
        if (x + 10 < tft180_width_max - 18) {
            tft180_show_int(x, y - 10, digit , 2);
        }
        else {
            tft180_show_int(x - 10, y - 5, digit , 2);
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     跟踪小车路径
// 参数说明     bias           坐标偏置
// 返回参数     void
// 使用示例     car_track(temp_bias);
// 备注信息     需要的时候进行调用，要放在while(1)中
//-------------------------------------------------------------------------------------------------------------------
void car_track(point* bias) {
    point t;
    get_point(navigation.x_cur, navigation.y_cur, &t, bias, start_pos);
    tft180_draw_point(t.x, t.y, RGB565_BLUE);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     可视化对应的点位
// 参数说明     void
// 返回参数     void
// 使用示例     position_show();
// 备注信息     需要的时候进行调用，一般要放在while(1)中
//-------------------------------------------------------------------------------------------------------------------
void position_show(void) {
    // 用于绘制点位周围一圈点位要用的偏置数组
    static int dx[5] = {-2, -1, 0, 1, 2}, dy[5] = {-2, -1, 0, 1, 2};
    tft180_clear();
    auto_adapt_size();
    point cur, last, bias;
    // bias 表示起点方框的长宽
    bias.x = 10, bias.y = 10;

    // 用 last 表示惯导坐标系中(0, 0)坐标在屏幕坐标系中的坐标，表示上一个cur值
    // whole_bias 第309行有解释
    get_point(0, 0, &last, &whole_bias, start_pos);

    // 根据绘制起点的方框
    if (start_pos == RIGHT_BOTTOM) {
        tft180_draw_line(last.x, last.y, last.x - bias.x, last.y, RGB565_RED);
        tft180_draw_line(last.x - bias.x, last.y, last.x - bias.x, last.y - bias.y, RGB565_RED);
        tft180_draw_line(last.x - bias.x, last.y - bias.y, last.x, last.y - bias.y, RGB565_RED);
        tft180_draw_line(last.x, last.y - bias.y, last.x, last.y, RGB565_RED);
    }
    else {
        tft180_draw_line(last.x, last.y, last.x + bias.x, last.y, RGB565_RED);
        tft180_draw_line(last.x + bias.x, last.y, last.x + bias.x, last.y - bias.y, RGB565_RED);
        tft180_draw_line(last.x + bias.x, last.y - bias.y, last.x, last.y - bias.y, RGB565_RED);
        tft180_draw_line(last.x, last.y - bias.y, last.x, last.y, RGB565_RED);
    }

    // bias 整体减半得到所绘制的起点方框的中心点偏置
    // whole_bias 同上
    bias.x = bias.x / 2 + whole_bias.x, bias.y = bias.y / 2 + whole_bias.y;
    tmp_bias.x = bias.x, tmp_bias.y = bias.y;

    // 遍历所有的点位，再屏幕中绘制出相应的点位，并且将相邻的点位连线
    for (int i = 0; i <= navigation.pos_index; i ++) {
        get_point(car_position[i].x_cur, car_position[i].y_cur, &cur, &bias, start_pos);
        tft180_draw_line(last.x, last.y, cur.x, cur.y, RGB565_RED);
        // 画点位的黑框
        for (int k = 0; k < 5; k ++) {
            for (int j = 0; j < 5; j ++) {
                uint16 x = cur.x + dx[k], y = cur.y + dy[j];
                if (i && i != navigation.pos_index && dx[k] == 0 && dy[j] == 0) {
                    draw_digit(x, y, i);
                }
                // 越界判断
                if (x < 0 || x >= tft180_width_max || y < 0 || y >= tft180_height_max) continue;
                // 对当前选中的点进行颜色上的区分
                if (cur_select == i) {
                   tft180_draw_point(x, y, RGB565_YELLOW);
                }
                else {
                    tft180_draw_point(x, y, RGB565_BLACK);
                }
            }
        }
        // 更新last
        last = cur;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     进行点位的选择
// 参数说明     void
// 返回参数     void
// 使用示例     position_select();
// 备注信息     一般要放在while(1)中，配合菜单使用
//-------------------------------------------------------------------------------------------------------------------
void position_select(void) {
    if (key_state[3] == KEY_SHORT_PRESS) {
        if (cur_select < navigation.pos_index - 1) {
            cur_select += 1;
        }
        else {
            cur_select = 1;
        }
        position_update = 1;
    }
    else if (key_state[4] == KEY_SHORT_PRESS) {
        if (cur_select > 1) {
            cur_select -= 1;
        }
        else {
            cur_select = navigation.pos_index - 1;
        }
        position_update = 1;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     判断操作并执行操作
// 参数说明     void
// 返回参数     void
// 使用示例     operation_detect_handle();
// 备注信息     一般要放在while(1)中，配合菜单使用
//-------------------------------------------------------------------------------------------------------------------
void operation_detect_handle(void) {
    // 定义菜单的数据
    static menu_item point_menu[] = {
            {00, "cancel", cancel,          NULL},
            {01, "modify", modify_position, NULL},
            {02, "add",    add_position,    NULL},
            {03, "delete", del_position,    NULL},
    };
    if (key_state[2] == KEY_SHORT_PRESS) {
        tft180_clear();
        int highlight_col = 0, parent_menu_id = 0;
        // 显示子菜单
        int menu_item_count = show_sub_menu(point_menu, sizeof(point_menu) / sizeof(point_menu[0]), parent_menu_id, highlight_col);
        while (TRUE) {
            if (key_switch())
            {
                if (key_state[0] == KEY_SHORT_PRESS) {
                    highlight_col--;
                    if (highlight_col < 0) highlight_col = menu_item_count - 1;
                }
                else if (key_state[1] == KEY_SHORT_PRESS)
                {
                    highlight_col++;
                    if (highlight_col >= menu_item_count) highlight_col = 0;
                }
                else if (key_state[2] == KEY_SHORT_PRESS)
                {
                    // 执行相应的函数
                    current_menu_item->menu_action();
                    return;
                }
                // 显示子菜单
                menu_item_count = show_sub_menu(point_menu, sizeof(point_menu) / sizeof(point_menu[0]), parent_menu_id, highlight_col);
                // 清除所有的按键状态
                key_clear_all_state();
            }
        }
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     菜单中的执行询问操作
// 参数说明     void
// 返回参数     void
// 使用示例     query_position();
// 备注信息     一般要放在while(1)中，配合菜单使用
//-------------------------------------------------------------------------------------------------------------------
void query_position(void) {
    // 显示位置信息
    position_show();
    while (TRUE) {
        // 点位选择
        position_select();
        // 操作判断
        operation_detect_handle();
        // 小车追踪
        car_track(&tmp_bias);
        // 根据是否有点位变动来刷新，提升性能
        if (position_update == 1) {
            position_show();
            position_update = 0;
        }
        if (key_state[2] == KEY_LONG_PRESS) {
            return;
        }
        if (key_state[5] == KEY_SHORT_PRESS) {
            Start_Flag = !Start_Flag;
            system_delay_ms(50);
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     惯性导航开始
// 参数说明     void
// 返回参数     void
// 使用示例     navigationStart();
// 备注信息     开始导航的时候调用，注意要保证开始导航的位置和踩点的时候的起点位置相同
//-------------------------------------------------------------------------------------------------------------------
void navigationStart(void) {
    tft180_clear();
    cur_car_mode = INERTIAL_NAVIGATION;
    navigation.start_flag = 1;
    navigation.x_cur = 0;
    navigation.y_cur = 0;
    query_position();
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     导航方向判断
// 参数说明     void
// 返回参数     void
// 使用示例     direction_recognition();
// 备注信息     根据坐标来判断
//-------------------------------------------------------------------------------------------------------------------
void direction_recognition(void) {
    if (!navigation.start_flag || navigation.clear_flag) return;

    float rad = ANGLE_TO_RAD(navigation.cur_yaw);

    float x = navigation.x_cur + sin(rad) * 5;
    float y = navigation.y_cur + cos(rad) * 5;
    if (get_distance(navigation.x_cur, navigation.y_cur, navigation.x_set, navigation.y_set) >
        get_distance(x, y, navigation.x_set, navigation.y_set)) {
        if (navigation.speed_direction == -1) {
            // 方向从 -1 变成 1, 所以方向翻转了
            navigation.reverse_flag = 1;
        }
        navigation.speed_direction = 1;
    }
    else {
        // 同上
        if (navigation.speed_direction == 1) {
            navigation.reverse_flag = 1;
        }
        navigation.speed_direction = -1;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     重置惯性导航的参数
// 参数说明     void
// 返回参数     void
// 使用示例     navigationReset();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void navigationReset(void) {
    cur_car_mode = REMOTE_CONTROL;
    navigation.bdc_speed = 0;
    navigation.bdc_speed_max = 0;
    navigation.offset = 0;
    angleUsed = 0;
    angle_offset = 0;
    navigation.back_home = 0;
    navigation.start_flag = 0;
    navigation.reverse_flag = 0;
    navigation.cur_index = 0;
    lit_idx = 0;
    navigation.x_cur = 0;
    navigation.y_cur = 0;
    navigation.x_set = 0;
    navigation.y_set = 0;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     更新欧拉角
// 参数说明     void
// 返回参数     void
// 使用示例     eulerAngleUpdate();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void eulerAngleUpdate(void) {
    eulerAngle_yaw = M_eulerAngle_yaw;
    eulerAngle_yaw_total = M_eulerAngle_yaw_total;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     惯导坐标系刷新
// 参数说明     void
// 返回参数     void
// 使用示例     INS_Update();
// 备注信息     放在 2ms 中断中
//-------------------------------------------------------------------------------------------------------------------
void INS_Update(void) {
    // 计算当前的yaw角度
    navigation.cur_yaw = eulerAngle_yaw_total - navigation.start_yaw;

    // 计算惯性导航当前距离
    navigation.INS_cur_distance = encoder_bdc_total;

    float rad = ANGLE_TO_RAD(navigation.cur_yaw);
    float distance = encoder_bdc / 150 * 50 * 0.004;

    // 计算当前的 x 和 y
    navigation.x_cur += sin(rad) * distance;
    navigation.y_cur += cos(rad) * distance;

    //计算当前坐标与目标坐标的角度
    if (cur_car_mode == INERTIAL_NAVIGATION) {
        float targetRad = atan2((navigation.x_set - navigation.x_cur), (navigation.y_set - navigation.y_cur));
        float targetAngle = RAD_TO_ANGLE(targetRad);

        navigation.INS_set_yaw_update = targetAngle;

        // 进行积分，防止突变
        float delta = navigation.INS_set_yaw_update - navigation.INS_set_yaw_old;

        while (delta > 180) delta = delta - 360;
        while (delta < -180) delta = delta + 360;

        navigation.INS_set_yaw_total += delta;

        // 只能正着走
        while (navigation.INS_set_yaw_total - navigation.cur_yaw >= 180) {
            navigation.INS_set_yaw_total -= 360;
        }
        while (navigation.INS_set_yaw_total - navigation.cur_yaw < -180) {
            navigation.INS_set_yaw_total += 360;
        }

        if (navigation.back_car_flag) {
            // 转最小的角度，此时方向判断要开启，否则可能会向反方向跑
            while (navigation.INS_set_yaw_total - navigation.cur_yaw > 90) {
                navigation.INS_set_yaw_total -= 180;
            }
            while (navigation.INS_set_yaw_total - navigation.cur_yaw < -90) {
                navigation.INS_set_yaw_total += 180;
            }
        }

        // 更新 navigation.INS_set_yaw_old
        navigation.INS_set_yaw_old = navigation.INS_set_yaw_total;
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     参数刷新
// 参数说明     void
// 返回参数     void
// 使用示例     INS_Update();
// 备注信息     放在 2ms 中断中
//-------------------------------------------------------------------------------------------------------------------
void Para_Update(void) {
    // 计算当前的yaw角度
    navigation.cur_yaw = eulerAngle_yaw_total - navigation.start_yaw;
    // 计算惯性导航当前距离
    navigation.INS_cur_distance = encoder_bdc_total;

    if (cur_car_mode == INERTIAL_NAVIGATION) {
        // 计算当前角度和目标角度的偏差
        navigation.offset = -(navigation.cur_yaw - navigation.INS_set_yaw_total);
        if (navigation.start_flag) {
            float distance = get_distance(navigation.x_cur, navigation.y_cur, navigation.x_set, navigation.y_set);
            // 根据转向角和当前点和目标点之间的距离进行速度决策
            if (fabs(angle_offset) > 20 || navigation.reverse_flag) {
                navigation.bdc_speed_max = navigation.speed1;
            }
            else if (fall_flag || distance <= 3 * ERROR_POSITION_ACCEPT || hasFoundTargetBoard) {
                navigation.bdc_speed_max = navigation.speed2;
            }
            else if (distance <= 6 * ERROR_POSITION_ACCEPT) {
                navigation.bdc_speed_max = navigation.speed3;
            }
            else {
                navigation.bdc_speed_max = navigation.speed4;
            }
        }
    }

    // 更新速度
    if (navigation.speed_direction == 1) {
        navigation.bdc_speed = navigation.bdc_speed_max;
    }
    else if (navigation.speed_direction == -1) {
        navigation.bdc_speed = -navigation.bdc_speed_max;
    }
}


//-------------------------------------------------------------------------------------------------------------------
// 函数简介     惯性导航点位的切换
// 参数说明     void
// 返回参数     void
// 使用示例     Navigation_Handler();
// 备注信息     放在 2ms 中断中
//-------------------------------------------------------------------------------------------------------------------
void Navigation_Handler(void) {
    if (navigation.start_flag != 1) return;
    if (navigation.back_home) {
        if (get_distance(navigation.x_cur, navigation.y_cur, navigation.x_set, navigation.y_set) <= ERROR_POSITION_ACCEPT)
        {
            Start_Flag = 0;
        }
        return;
    }

    if (navigation.cur_index >= navigation.pos_index) {
        Go_home();
    }
    else {
        // 如果已经到达了相应的点位(有一定的误差)
        if (!navigation.cur_index ||
            (get_distance(navigation.x_cur, navigation.y_cur, navigation.x_set, navigation.y_set) <= ERROR_POSITION_ACCEPT)) {
            navigation.cur_index ++;
            navigation.x_set = car_position[navigation.cur_index].x_cur;
            navigation.y_set = car_position[navigation.cur_index].y_cur;
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将导航位置信息写到flash中
// 参数说明     void
// 返回参数     void
// 使用示例     navi_savePositionToFlash();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void navi_savePositionToFlash(void) {
    zf_assert(navigation.pos_index < EEPROM_PAGE_LENGTH / 2);

    if(flash_check(0, FLASH_POSITION_PAGE))                       // 判断是否有数据
        flash_erase_page(0, FLASH_POSITION_PAGE);                 // 擦除这一页
    flash_buffer_clear();                                         // 将缓存清空

    // 位置数量存储到第一个位置
    flash_union_buffer[0].uint16_type = navigation.pos_index;
    flash_union_buffer[1].int32_type = sp_point_index;

    // 开始存储数据
    uint16 cur = 1;
    for (cur = 1; cur <= navigation.pos_index; cur ++) {
        flash_union_buffer[2 * cur].float_type = car_position[cur - 1].x_cur;
        flash_union_buffer[2 * cur + 1].float_type = car_position[cur - 1].y_cur;
    }

    // 存储灯光秀的点位
    for (int i = 0; i < sp_point_index; i ++) {
        flash_union_buffer[2 * cur + i].int32_type = light_points[i];
    }

    // 写入对应的flash扇区
    if((flash_write_page_from_buffer(0, FLASH_POSITION_PAGE)) == 0) {
        tft180_clear();
        tft180_show_string(0, 0, "flash success!!");
        system_delay_ms(50);
    }
    else {
        tft180_clear();
        tft180_show_string(0, 0, "flash failed!! please try again!!");
        system_delay_ms(50);
    }

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取flash中的位置信息
// 参数说明     void
// 返回参数     void
// 使用示例     navi_readPositionFromFlash();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void navi_readPositionFromFlash(void) {
    flash_buffer_clear();
    flash_read_page_to_buffer(0, FLASH_POSITION_PAGE); // 将数据从flash读到缓冲区

    // 开始读取数据
    navigation.pos_index = flash_union_buffer[0].uint16_type;
    sp_point_index = flash_union_buffer[1].int32_type;

    uint16 cur = 1;
    for (cur = 1; cur <= navigation.pos_index; cur ++) {
        car_position[cur - 1].x_cur = flash_union_buffer[2 * cur].float_type;
        car_position[cur - 1].y_cur = flash_union_buffer[2 * cur + 1].float_type;
    }

    // 读取灯关秀的下标信息
    for (int i = 0; i < sp_point_index; i ++) {
        light_points[i] = flash_union_buffer[2 * cur + i].int32_type;
    }

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     将flash中的数据全部清空
// 参数说明     void
// 返回参数     void
// 使用示例     navi_clearAllPos();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void navi_clearAllPos(void) {
    if(flash_check(0, FLASH_POSITION_PAGE))                       // 判断是否有数据
        flash_erase_page(0, FLASH_POSITION_PAGE);                 // 擦除这一页

    flash_buffer_clear();
    navigation.pos_index = flash_union_buffer[0].uint16_type;
    sp_point_index = flash_union_buffer[1].int32_type;

    // 清空flash中的数据
    flash_union_buffer[0].uint16_type = 0;
    flash_union_buffer[1].int32_type = 0;

    uint16 cur = 1;
    for (cur = 1; cur <= navigation.pos_index; cur ++) {
        flash_union_buffer[2 * cur].float_type = 0.0;
        flash_union_buffer[2 * cur + 1].float_type = 0.0;
    }

    for (int i = 0; i < sp_point_index; i ++) {
        flash_union_buffer[2 * cur + i].int32_type = light_points[i];
    }

    if((flash_write_page_from_buffer(0, FLASH_POSITION_PAGE)) == 0)
    {
        tft180_clear();
        tft180_show_string(0, 0, "clear success!!");
        system_delay_ms(50);
    }
    else {
        tft180_clear();
        tft180_show_string(0, 0, "clear failed!! please try again!!");
        system_delay_ms(50);
    }

}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     读取当前的位置信息
// 参数说明     void
// 返回参数     void
// 使用示例     GetPosInfo();
// 备注信息     踩点的时候调用
//-------------------------------------------------------------------------------------------------------------------
void GetPosInfo(void) {
    static uint8 sp_index = 0;
    if (navigation.get_index >= MAX_POSITION_COUNT) return;

    // 记录的点位要满足：连续两个点在允许的误差之外，并且不是第一个点，第一个点位要在出发点采集
    if (navigation.get_index && \
       (get_distance(navigation.x_cur, navigation.y_cur,
               car_position[navigation.get_index - 1].x_cur, car_position[navigation.get_index - 1].y_cur) <= ERROR_POSITION_ACCEPT)) {
        return;
    }
    Buzzer_on();
    system_delay_ms(100);
    Buzzer_off();

    get_pos(navigation.get_index);
    // 灯光秀的特殊点位打点
    if (sp_flag) {
        light_points[sp_index ++ ] = navigation.get_index + 1;
        sp_point_index = sp_index;
    }
    navigation.get_index ++;
    navigation.pos_index = navigation.get_index;
    tft180_show_int(50, 50, navigation.get_index, 2);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     回库
// 参数说明     void
// 返回参数     void
// 使用示例     Go_home();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void Go_home(void) {
    cur_car_mode = INERTIAL_NAVIGATION;
    navigation.back_home = 1;
    navigation.x_set = navigation.x_start;
    navigation.y_set = navigation.x_start;
    navigation.INS_set_yaw = eulerAngle_yaw_total;
    navigation.INS_set_yaw_total = navigation.start_yaw;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     惯导debug
// 参数说明     void
// 返回参数     void
// 使用示例     navigation_para_show();
// 备注信息
//-------------------------------------------------------------------------------------------------------------------
void navigation_para_show(void) {
    tft180_show_float(0, 0, navigation.offset, 3, 2);
    tft180_show_float(60, 0, eulerAngle.yaw, 3, 2);
    tft180_show_float(60, 20, navigation.INS_set_yaw_total, 3, 2);
    tft180_show_float(60, 40, navigation.INS_set_yaw, 3, 2);
    tft180_show_float(60, 60, navigation.cur_yaw, 3, 2);
    tft180_show_int(0, 20, navigation.cur_index, 2);
    tft180_show_int(0, 40, navigation.pos_index, 2);
    tft180_show_float(0, 80, navigation.x_cur, 3, 2);
    tft180_show_float(0, 100, navigation.y_cur, 3, 2);
    tft180_show_float(0, 120, navigation.x_set, 3, 2);
    tft180_show_float(0, 140, navigation.y_set, 3, 2);
}
//#pragma section all restore
