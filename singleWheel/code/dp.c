#include "dp.h"

my_offset offset_variable;
float angle_offset = 0;
float speed = 0;
float speed_test = 150;
float Dynamic_zero_set = 0.0f;
float yaokong_speed,yaokong_angle = 0.0;
float K_zero_dot = 0;
float rtk_speed = 0;
int slow_flag = 0;
int fangxiang_flag = 3;
float zero_set = 0;
float zero_Error = 0;
float angleUsed = 0;
uint16 show_index = 0;
float K1 = 0.5, K2 = 0.15; // 双变量调节动态零点

void dataprocessing(void)
{
    yaokong_getvalue();
    speed_offset_handle(yaokong_speed + navigation.bdc_speed);
    anlgle_offset_handle(yaokong_angle, image_offset, navigation.offset, 0);
    slow_check();
    fangxiang_check();
}

void anlgle_offset_handle(float yaokong_angle_offset,float image_offset,float navigation_offset,float RTK_offset)
{
    float k1 = 1, k2 = 3, k3 = 1, k4 = 1, offset_low_pass = 0.6;

    offset_variable.yaokong_angle_offset =     k1 * yaokong_angle_offset;        //-90-90
    offset_variable.image_offset         =     k2 * image_offset;                //-180-180
    offset_variable.navigation_offset    =     k3 * navigation_offset;           //-720-720
    offset_variable.RTK_offset           =     k4 * RTK_offset;

//    angle_offset = offset_variable.yaokong_angle_offset+offset_variable.image_offset+offset_variable.navigation_offset+offset_variable.RTK_offset;

    float get_offset = offset_variable.yaokong_angle_offset+offset_variable.image_offset+offset_variable.navigation_offset+offset_variable.RTK_offset;

    angle_offset = angle_offset * offset_low_pass + get_offset * (1 - offset_low_pass);

//    if (speed_reversal_flag == -1) {
//        angle_offset = offset_variable.navigation_offset;
//    }
    if (navigation.reverse_flag) {
        angle_offset = 0;
    }
    else if (!navigation.back_home && navigation.clear_flag) {
        angle_offset = offset_variable.image_offset;
    }
//    if (navigation_start_flag && fabs(angle_offset) > 20) {
//        bdc_speed = bdc_speed_max = 120;
//    }
    angle_offset = fclip(angle_offset,-80, 80);
}
void speed_offset_handle(float speed_in)
{
    speed = speed_in;

    speed = fclip(speed,-500,500);
}

// 0.5 0.15

float Dynamic_zero_cale(void) {

#ifdef DOUBLE_VARIABLE
    static float g = 9.8;                                      // 重力加速度
    float alpha = ANGLE_TO_RAD(angleUsed);                     // 实际偏差角度(RAD)
    float real_speed_square = Encoder_C * Encoder_C * 0.000044;// 实际速度

    zero_Error = sin(alpha * K2) * g + real_speed_square * (alpha * K2) / K1; //先给恒定速度调稳，再加速度环闭环

    if (zero_Error > 9) zero_Error = 9;
    if (zero_Error < -9) zero_Error = -9;


#else
    zero_Error = K_zero_dot * 0.01 * angleUsed; //先给恒定速度调稳，再加速度环闭环
#endif

//    if(zero_set < zero_Error)       zero_set = zero_set + 0.008;      //0.008   0.02
//    else if(zero_set > zero_Error)  zero_set = zero_set - 0.008;

//    zero_Error = zero_set;

//    if(slow_flag)  zero_Error = 0;

    return zero_Error;
}


void yaokong_getvalue(void)
{
    if(lora3a22_state_flag == 1)
    {
        yaokong_angle = -Map(lora3a22_uart_transfer.joystick[2],-2000,2000,-90,90);
        yaokong_speed = Map(lora3a22_uart_transfer.joystick[1],-2000,2000,-500,500);
    }
    else
    {
        yaokong_speed = 0;
        yaokong_angle = 0;
    }
}

void yaokong_check(void)
{
    lora3a22_response_time++;
    if (lora3a22_response_time > 500 / 20)   //500ms 婵炲备鍓濆﹢渚�骞掗妷銉ョ秬闁稿﹥甯楅弳鐔煎箲椤旂厧鐏查柡鍌ゅ幒缂嶅懘宕ｉ幋锔兼嫹娴ｄ警浼傜�殿喖鍊搁悥锟�
    {
        lora3a22_state_flag = 0;                     //闂侇兙鍎茬敮鍫曞闯閵娧冃﹂柟顑挎缂嶅懎銆掗崨瀛樼ォ
        lora3a22_response_time = 0;
    }
}
void slow_check(void){
    if(abs(Encoder_C) < 120){    slow_flag = 1; }
    else                    {    slow_flag = 0; }
}

void fangxiang_check(void){
    if(Encoder_C > 50)          fangxiang_flag = 1;//闁告挸绉风换锟�
    else if(Encoder_C < -50)    fangxiang_flag = 2;//闁告艾姘﹂崥锟�
    else                        fangxiang_flag = 3;//闁告鍠庡﹢锟�
}







