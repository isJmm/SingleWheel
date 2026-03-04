#include "init.h"

uint8 get_point_flag = 0;

void Buzzer_on(void)  { gpio_set_level(P21_5, 1); }
void Buzzer_off(void) { gpio_set_level(P21_5, 0); }

void Init(void)
{
    //PID初始化
    PID_all_init();
//     逆透视初始化
//    ImagePerspective_Init();

    // imu_init();
    ADIS16505_init();
    get_gyroOffset();

    for (int i = 0; i < 1000; i ++) {
        ADIS16505_task();
    }
    start_angle = M_eulerAngle_yaw;

    //屏幕初始化
    tft180_set_font(TFT180_8X16_FONT);
    tft180_set_color(RGB565_RED,RGB565_WHITE);
    tft180_init();

    //flash参数读取
//    Clear_All_Parameter();
//    Save_Parameter();// 用代码参数,没事别打开
    Read_Parameter();
    Start_Flag = 0;
    navi_readPositionFromFlash();

    //无线串口初始化
//   wireless_uart_init();

    //遥控器接收机初始化
    lora3a22_init();

    //按键初始化
    key_init(10);
    gpio_init(P20_12, GPI, GPIO_HIGH, GPI_FLOATING_IN);
    gpio_init(P20_11, GPI, GPIO_HIGH, GPI_FLOATING_IN);
    gpio_init(P20_7, GPI, GPIO_HIGH, GPI_FLOATING_IN);
    gpio_init(P20_6, GPI, GPIO_HIGH, GPI_FLOATING_IN);
    gpio_init(P20_2, GPI, GPIO_HIGH, GPI_FLOATING_IN);
    gpio_init(P21_7, GPI, GPIO_HIGH, GPI_FLOATING_IN);
    gpio_init(P20_13, GPI, GPIO_HIGH, GPI_FLOATING_IN);
    gpio_init(P20_14, GPI, GPIO_HIGH, GPI_FLOATING_IN);
    get_point_flag = gpio_get_level(P20_13);
    use_left_bottom = gpio_get_level(P20_14);

//    if (!get_point_flag && !use_left_bottom)     { speed1 = 160, speed2 = 180, speed3 = 240, speed4 = 320; }
//    else if (get_point_flag && !use_left_bottom) { speed1 = 160, speed2 = 180, speed3 = 260, speed4 = 350; }
//    else if (!get_point_flag && use_left_bottom) { speed1 = 160, speed2 = 180, speed3 = 280, speed4 = 375; }
//    else if (get_point_flag && use_left_bottom)  { speed1 = 160, speed2 = 180, speed3 = 300, speed4 = 400; }


    //蜂鸣器初始化
    gpio_init(P21_5, GPO, GPIO_LOW, GPO_PUSH_PULL);


    //编码器初始化
    encoder_quad_init(TIM2_ENCODER, TIM2_ENCODER_CH1_P33_7, TIM2_ENCODER_CH2_P33_6); //左动量轮
    encoder_quad_init(TIM6_ENCODER, TIM6_ENCODER_CH1_P20_3, TIM6_ENCODER_CH2_P20_0); //右动量轮
    encoder_quad_init(TIM3_ENCODER, TIM3_ENCODER_CH1_P02_6, TIM3_ENCODER_CH2_P02_7); //行进轮

    //电机初始化
    pwm_init(ATOM0_CH2_P33_11, 17000, 10000); //左动量轮
    gpio_init(P33_5, GPO, GPIO_HIGH, GPO_PUSH_PULL); //方向
    gpio_init(P33_4, GPO, GPIO_LOW, GPO_PUSH_PULL); //刹车
    pwm_init(ATOM0_CH4_P22_3, 17000, 10000); //右动量轮
    gpio_init(P22_1, GPO, GPIO_HIGH, GPO_PUSH_PULL); //方向
    gpio_init(P23_1, GPO, GPIO_LOW, GPO_PUSH_PULL); //刹车
    pwm_init(ATOM0_CH7_P00_8, 17000, 0); //行进轮
    gpio_init(P00_9, GPO, GPIO_HIGH, GPO_PUSH_PULL); //方向

    // 摄像头初始化
//    uart_init (SCC8660_COF_UART, SCC8660_COF_BAUR, SCC8660_COF_UART_RX, SCC8660_COF_UART_TX);
    while(1)
    {
        if(scc8660_init())
            tft180_show_string(0, 80, "scc8660 reinit.");
        else
            break;
        system_delay_ms(500);                                                   // 短延时快速闪灯表示异常
    }

    // 惯性导航初始化
    cur_car_mode = REMOTE_CONTROL;

    if (use_left_bottom) start_pos = LEFT_BOTTOM;
    else                 start_pos = RIGHT_BOTTOM;
    INS_Init();

    Buzzer_on();
    system_delay_ms(200);
    Buzzer_off();

    //pit中断初始化
    pit_ms_init(CCU60_CH0, 1); // control
    pit_ms_init(CCU60_CH1, 5); //
    pit_ms_init(CCU61_CH0, 2); // 惯导处理的中断

    //卡尔曼滤波
    KalmanCreate(&Zero_Kalman, 10, 200);
}

