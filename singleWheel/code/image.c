#include "image.h"

#pragma section all "cpu1_dsram"

#define min(a, b) ((a) < (b) ? (a) : (b))
#define max(a, b) ((a) < (b) ? (b) : (a))

const float fGamma = 1.5;

// 二值化的图像
uint16 Threshold_Image[IMAGE_H][IMAGE_W];
//压缩的图像
uint16 Image_Used[IMAGE_H][IMAGE_W];

// 逼近实际的摄像头偏差的变量
float to_image_offset = 0;
// 实际的摄像头偏差
float image_offset = 0;
uint8 resetFlag = 0;
uint8 fall_flag = 0;
float image_flag = 0; // 表示是不是要使用图像

int hasFoundTargetBoard = 0;

// 目标板RGB颜色三个通道限制
color_limits redRgbLimits = {31, 25, 10, 0, 10, 0};
color_limits yellowRgbLimits = {25, 15, 45, 25, 10, 0};

// 目标板HSV颜色三个通道限制
color_limits redHsvLimits = {20.0, 0.0, 100, 60, 100, 60};
color_limits yellowHsvLimits = {55.0, 35.0, 100, 50, 100, 50};


Objects cone        = { RGB565_RED,    {20.0, 0.0, 100, 60, 100, 60},  {31, 25, 10, 0, 10, 0} };
Objects targetboard = { RGB565_YELLOW, {60.0, 40.0, 100, 50, 100, 50}, {25, 15, 45, 25, 10, 0}};

uint16 check_color(Objects* target, color_channels* color, enum IMAGE_MODE image_mode) {
    if (image_mode == HSV) {
        if (
            ((color->first >= target->hsv_limits.first_min &&
                    color->first <= target->hsv_limits.first_max)) &&
            (color->second >= target->hsv_limits.second_min &&
                    color->second <= target->hsv_limits.second_max) &&
            (color->third >= target->hsv_limits.third_min &&
                    color->third <= target->hsv_limits.third_max)
           )
        {
            return 1;
        }
    }
    else if (image_mode == RGB) {
        if (
            (color->first >= target->rgb565_limits.first_min &&
                    color->first <= target->rgb565_limits.first_max) &&
            (color->second >= target->rgb565_limits.second_min &&
                    color->second <= target->rgb565_limits.second_max) &&
            (color->third >= target->rgb565_limits.third_min &&
                    color->third <= target->rgb565_limits.third_max)
           )
        {
            return 1;
        }
    }
    return 0;
}

void GetRgb565(uint16 data, color_channels* rgb) {
    rgb->first = getRed(data); // (data >> 11) & 0x1f;
    rgb->second = getGreen(data); // (data >> 5) & 0x3f;
    rgb->third = getBlue(data); // data & 0x1f;
}

void Rgb565ToHsv(color_channels* rgb, color_channels* hsv) {
    float R = 1.0 * (rgb->first) / 31;
    float G = 1.0 * (rgb->second) / 63;
    float B = 1.0 * (rgb->third) / 31;
    float h, s, v;

    float minRGB = min_float(R, min_float(G, B));
    float maxRGB = max_float(R, max_float(G, B));
    float delta = maxRGB - minRGB;

    // V
    v = maxRGB;

    // S
    if (v > 0) {
        s = 1.0 * delta / v;
    }
    else {
        s = 0.0;
        h = 0.0; // undefined
        return;
    }

    // H
    if (maxRGB == R) {
        h = 1.0 * 60 * (G - B) / delta;
    }
    else if (v == G) {
        h = 120 + 1.0 * 60 * (B - R) / delta;
    }
    else {
        h = 240 + 1.0 * 60 * (R - G) / delta;
    }

    if (h < 0) {
        h += 360;
    }
    s = s * 100; // 归一化到[0, 100]
    v = v * 100;  // 归一化到[0, 100]
    hsv->first = h, hsv->second = s, hsv->third = v;
}

void Rgb565ToHsl(color_channels* rgb, color_channels* hsl) {
    float R = 1.0 * (rgb->first) / 31;
    float G = 1.0 * (rgb->second) / 63;
    float B = 1.0 * (rgb->third) / 31;

    float minRGB = min_float(R, min_float(G, B));
    float maxRGB = max_float(R, max_float(G, B));
    float delta = maxRGB - minRGB, sum = minRGB + maxRGB;

    if (maxRGB == minRGB) {
        hsl->first = 0;
    }
    else if (maxRGB == R && G >= B) {
        hsl->first = 60 * (G - B) / delta;
    }
    else if (maxRGB == R && G < B) {
        hsl->first = 60 * (G - B) / delta + 360;
    }
    else if (maxRGB == G) {
        hsl->first = 60 * (B - R) / delta + 120;
    }
    else if (maxRGB == B) {
        hsl->first = 60 * (R - G) / delta + 240;
    }

    hsl->third = sum / 2.0;

    if (hsl->second == 0 || maxRGB == minRGB) {
        hsl->third = 0;
    }
    else if (hsl->second > 0 && hsl->second <= 0.5) {
        hsl->third = delta / sum;
    }
    else {
        hsl->third = delta / (2 - sum);
    }
}

color_channels HslToRgb565(color_channels* hsl) {
    float chroma = (1.0 - fabs(2.0 * hsl->third - 1.0)) * hsl->second;
    float h_prime = fmod(hsl->first / 60.0, 6.0);
    float x = chroma * (1.0 - fabs(fmod(h_prime, 2.0) - 1.0));
    float m = hsl->third - chroma / 2.0;

    float r, g, b;
    if (0.0 <= h_prime && h_prime < 1.0) {
        r = chroma; g = x; b = 0.0;
    } else if (1.0 <= h_prime && h_prime < 2.0) {
        r = x; g = chroma; b = 0.0;
    } else if (2.0 <= h_prime && h_prime < 3.0) {
        r = 0.0; g = chroma; b = x;
    } else if (3.0 <= h_prime && h_prime < 4.0) {
        r = 0.0; g = x; b = chroma;
    } else if (4.0 <= h_prime && h_prime < 5.0) {
        r = x; g = 0.0; b = chroma;
    } else if (5.0 <= h_prime && h_prime < 6.0) {
        r = chroma; g = 0.0; b = x;
    }
    r = (r + m) * 31; g = (g + m) * 63; b = (b + m) * 31;
    color_channels rgb = {r, g, b};
    return rgb;
}

uint16 FindAreaPoint(uint16 begin_x, uint16 begin_y, uint16 end_x, uint16 end_y, uint16 image[][IMAGE_W], uint16 Color) {
    uint16 whiteCnt = 0;
    for (uint16 i = begin_y; i < end_y; i ++) {
        for (uint16 j = begin_x; j < end_x; j ++) {
            if (image[i][j] == Color) {
                whiteCnt += 1;
            }
        }
    }
    return whiteCnt;
}

uint16 FindAllWhitePoint(uint16 image[][IMAGE_W]) {
    return FindAreaPoint(0, 0, IMAGE_W, IMAGE_H, image, RGB565_WHITE);
}

uint16 FindAllConePoint(uint16 image[][IMAGE_W]) {
    return FindAreaPoint(0, 0, IMAGE_W, IMAGE_H, image, cone.color);
}

uint16 FindConeOffset(uint16 image[][IMAGE_W]) {
    float res = 0;
    uint8 i, j;
    for (i = 0; i < IMAGE_H; i ++) {
        for (j = 0; j < IMAGE_W; j ++) {
            if (image[i][j] == cone.color) {
                int x = j - IMAGE_W / 2;
                res += Slope_Function(x, 2, 60);
            }
        }
    }
    return res / 100;
}

int16 FindMidOffset(void) {
    uint16 White_Point = FindAllWhitePoint(Threshold_Image);

    uint16 x1 = 0, x2 = IMAGE_W / 2, x3 = IMAGE_W;
    uint16 y1 = 0, y2 = IMAGE_H / 3, y3 = IMAGE_H * 2 / 3, y4 = IMAGE_H;

    int16 offset = 0;
    // 如果白点是多MIN_BLOB_AREA的，说明发现了目标板
    if (White_Point > MIN_BLOB_AREA) {
        if (hasFoundTargetBoard == 0) {
            navigation.clear_flag = 1;
        }
        hasFoundTargetBoard = 1;
        int16 offset1 = FindAreaPoint( x1,     y1,     x2,  y2, Threshold_Image, RGB565_WHITE) - \
                         FindAreaPoint(x2 + 1, y1,     x3,  y2, Threshold_Image, RGB565_WHITE);
        int16 offset2 = FindAreaPoint( x1,     y2 + 1, x2,  y3, Threshold_Image, RGB565_WHITE) - \
                         FindAreaPoint(x2 + 1, y2 + 1, x3,  y3, Threshold_Image, RGB565_WHITE);
        int16 offset3 = FindAreaPoint( x1,     y3 + 1, x2,  y4, Threshold_Image, RGB565_WHITE) - \
                         FindAreaPoint(x2 + 1, y3 + 1, x3,  y4, Threshold_Image, RGB565_WHITE);
        offset = offset1 / 2 + offset2 + offset3 / 2;
    }
    else { // 没有发现色块
        offset = 0;
        hasFoundTargetBoard = 0;
        navigation.clear_flag = 0;
    }
    // 防止数据不稳定导致的左右摇摆
    if (fabs(offset) <= 40) {
        offset = 0;
    }
    return offset / 4;
}

void SplitTargetColor (uint16 image[][IMAGE_W], enum IMAGE_MODE image_mode) {
    uint8 i, j;
    color_channels hsv, rgb;
    for (i = 0; i < IMAGE_H; i ++) {
        for (j = 0; j < IMAGE_W; j ++) {
            GetRgb565(image[i][j], &rgb);
            Rgb565ToHsv(&rgb, &hsv);
            if (check_color(&targetboard, &hsv, image_mode)) {
                Threshold_Image[i][j] = RGB565_WHITE;
                continue;
            }
            Threshold_Image[i][j] = RGB565_BLACK;
        }
    }
}

// 设定rgb565_data为当前目标颜色(HSV模式下)
void setColorToTarget(uint16 rgb565_data, color_limits* hsv_limits) {
    color_channels hsv, rgb;
    GetRgb565(rgb565_data, &rgb);
    Rgb565ToHsv(&rgb, &hsv);

    if(hsv.first > COLOR_H_RANGE) {
        hsv_limits->first_min = hsv.first - COLOR_H_RANGE;
    }
    else {
        hsv_limits->first_min = 0;
    }
    if(hsv.first < (360 - COLOR_H_RANGE)) {
        hsv_limits->first_max = hsv.first + COLOR_H_RANGE;
    }
    else {
        hsv_limits->first_max = 360;
    }

    if(hsv.second > COLOR_S_RANGE) {
        hsv_limits->second_min = hsv.second - COLOR_S_RANGE;
    }
    else {
        hsv_limits->second_min = 0;
    }
    if(hsv.second < (100 - COLOR_S_RANGE)) {
        hsv_limits->second_max = hsv.second + COLOR_S_RANGE;
    }
    else {
        hsv_limits->second_max = 100;
    }


    if(hsv.third > COLOR_V_RANGE) {
        hsv_limits->third_min = hsv.third - COLOR_V_RANGE;
    }
    else {
        hsv_limits->third_min = 0;
    }
    if(hsv.third < (100 - COLOR_V_RANGE)) {
        hsv_limits->third_max = hsv.third + COLOR_V_RANGE;
    }
    else {
        hsv_limits->third_max = 100;
    }
}

void Para_show(enum IMAGE_MODE image_mode) {
    color_channels rgb;
    uint16 color = Image_Used[IMAGE_H / 2][IMAGE_W / 2];
    GetRgb565(color, &rgb);

    if (image_mode == RGB) {
        tft180_show_string(SCC8660_W / 2, 0 , "R:");
        tft180_show_string(SCC8660_W / 2, 16, "G:");
        tft180_show_string(SCC8660_W / 2, 32, "B:");
        tft180_show_int(SCC8660_W / 2 + 20, 0, rgb.first, 2);
        tft180_show_int(SCC8660_W / 2 + 20, 16, rgb.second, 2);
        tft180_show_int(SCC8660_W / 2 + 20, 32, rgb.third, 2);
    }
    else if (image_mode == HSV){
        color_channels t;
        Rgb565ToHsv(&rgb, &t);
        tft180_show_int(SCC8660_W / 2, 0, t.first, 3);
        tft180_show_int(SCC8660_W / 2, 16, t.second, 3);
        tft180_show_int(SCC8660_W / 2, 32, t.third, 3);
    }

    tft180_show_string(0, SCC8660_H, "W:");
    tft180_show_int(20, SCC8660_H, FindAllWhitePoint(Threshold_Image), 3);
//    tft180_show_string(0, SCC8660_H + 10, "bfs:");
//    tft180_show_int(20, SCC8660_H + 10, bfsToCountColor(Threshold_Image), 3);

    tft180_show_string(45, SCC8660_H, "Off:");
//    tft180_show_int(80, SCC8660_H, FindMidOffset(), 4);
    tft180_show_int(80, SCC8660_H, findCentreOffset(Threshold_Image), 2);
#if 1
    // 显示原始图像
    tft180_show_rgb565_image(0, 0, Image_Used[0], IMAGE_W, IMAGE_H, IMAGE_W, IMAGE_H, 0);
    // 显示二值化图像
    tft180_show_rgb565_image(0, IMAGE_H, Threshold_Image[0], IMAGE_W, IMAGE_H, IMAGE_W, IMAGE_H, 0);
#endif
}

float findCentreOffset(uint16 image[][IMAGE_W]) {
    uint16 White_Point = FindAllWhitePoint(Threshold_Image);
    float offset = 0;
    // 如果白点是多MIN_BLOB_AREA的，说明发现了目标板
    if (White_Point > MIN_BLOB_AREA) {

        if (hasFoundTargetBoard == 0) {
            navigation.clear_flag = 1;
        }
        hasFoundTargetBoard = 1;

        float midTotal = 0;
        int whiteLine = 0;
        for (int r = 0; r < IMAGE_H; r ++) {
            int left = IMAGE_W + 1, right = -1;
            uint8 flag = 0;
            for (int c = 0; c < IMAGE_W; c ++) {
                if (image[r][c] == RGB565_WHITE) {
                    left = min(left, c);
                    right = max(right, c);
                    flag = 1;
                }
            }
            if (flag == 1) {
                midTotal += (left + right) / 2.0;
                whiteLine += 1;
            }
        }
        offset = IMAGE_W / 2 - midTotal / whiteLine;
    }
    else { // 没有发现色块
        offset = 0;
        if (hasFoundTargetBoard == 1) {
            navigation.clear_flag = 0;
        }
        hasFoundTargetBoard = 0;
    }
    return hasFoundTargetBoard ? offset : 0;
}


void SCC8660_Task(void) {
    if(scc8660_finish_flag)
    {
        // 压缩图像
        Compressimage();
        // 伽马矫正
//        GammaCorrectiom(Image_Used[0], IMAGE_W, IMAGE_H, fGamma, Image_Used[0]);
        // 分离色块
        SplitTargetColor(Image_Used, HSV);
        // 像素滤波
        Pixle_Filter(Threshold_Image);
        // bfs滤波
        bfsFilter(Threshold_Image);
        // 逆透视
//         warpPerspective(Threshold_Image);
        // 获取四个角的点
        get_corner_points(Threshold_Image, corner_points);
        // 如果是满足要求的矩形就找误差，否则直接设置成0
        if (shapeJudge(corner_points)) {
            image_offset = -findCentreOffset(Threshold_Image);
        }
        else {
            image_offset = 0;
            hasFoundTargetBoard = 0;
            navigation.clear_flag = 0;
        }

        scc8660_finish_flag = 0;
    }
}

#pragma section all restore
//
