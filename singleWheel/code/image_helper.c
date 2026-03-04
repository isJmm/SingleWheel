#include "image_helper.h"

#pragma section all "cpu1_dsram"

// begin ... 逆透视(未测试)
#define RESULT_ROW (IMAGE_H - 1) // 结果图行数
#define RESULT_COL (IMAGE_W - 1) // 结果图列数
#define USED_ROW IMAGE_H // 用于透视变换图像的行数
#define USED_COL IMAGE_W // 用于透视变换图像的列数
#define PER_IMG Threshold_Image // SimBinImage: 用于透视变换的图像
#define ImageUsed *PerImg_ip // *PerImg_ip定义使用的图像，ImageUsed为用于巡线和识别的图像
uint16_t *PerImg_ip[RESULT_ROW][RESULT_COL]; // 透视变换后的图像数组
point corner_points[4];
START_POS start_pos;
// end ... 逆透视

//-------------------------------------------------------------------------------------------------------------------
//  rgb565 转换成 rgb888
//  rgb888->R = (rgb565->R << 3) | (rgb565->R >> 2);
//  rgb888->G = (rgb565->G << 2) | (rgb565->G);
//  rgb888->B = (rgb565->B << 3) | (rgb565->B >> 2);
//-------------------------------------------------------------------------------------------------------------------

//全局数组：包含三个通道的gamma校正查找表
uint16 g_GammaLUT_R[32];
uint16 g_GammaLUT_G[64];
uint16 g_GammaLUT_B[32];
// fGamma算法的参数
//fGamma的值决定了输入图像和输出图像之间的灰度映射方式，即决定了是增强低灰度值区域还是增高灰度值区域。
//fGamma > 1时，图像的高灰度区域对比度得到增强。 可以提升比较暗的地方的细节
//fGamma < 1时，图像的低灰度区域对比度得到增强。 可以提升比较亮的地方的细节
//fGamma = 1时，不改变原图像

// 整个图像的RGB三个通道的平均值
float R_average, G_average, B_average;

// 配合bfsToCountColor使用的全局数组
uint16 visited[IMAGE_H][IMAGE_W]; // 表示是否已经访问过
uint16 queueTop, queueTail;
uint16 maxLable = 0;     // 最大元素的标签
point colorQueue[IMAGE_W * IMAGE_H + 10]; // bfs队列
// 定义偏移量
int dx[4] = {1, 0, -1, 0}, dy[4] = {0, 1, 0, -1};

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     找最大色块
// 参数说明     image[][] 二值化图像数组
// 参数说明     x,y       色块起点坐标
// 参数说明     color     要找的颜色
// 参数说明     lable     该色块的标签
// 返回参数     uint16    最大色块数量
// 使用示例     bfsToCountColor(image, r, c, RGB565_WHITE, lable)
//-------------------------------------------------------------------------------------------------------------------
static uint16 bfsToCountColor(uint16 image[IMAGE_H][IMAGE_W], uint16 x, uint16 y, uint16 color, uint16 lable) {
    queueTop = 0, queueTail = 0; // 队列的头和尾
    colorQueue[queueTop].x = x, colorQueue[queueTop].y = y; // 把当前颜色入队
    queueTop ++; // 元素加一
    visited[x][y] = lable; // 打上标签
    uint16 color_cnt = 0; // 记录当前标签颜色数量
    while (queueTop > queueTail) { // 如果还有元素
        point t = colorQueue[queueTail ++];
        color_cnt ++ ;
        uint16 cur_x = t.x, cur_y = t.y;
        // 对该坐标上下左右进行判断
        for (int k = 0; k < 4; k ++) {
            uint16 next_x = cur_x + dx[k], next_y = cur_y + dy[k];
            // 边界判断
            if (next_x >= IMAGE_H|| next_y >= IMAGE_W) continue;
            // 条件判断
            if (visited[next_x][next_y] || image[next_x][next_y] != color) continue;
            // 如果条件成立，对该坐标打上标签
            visited[next_x][next_y] = lable;
            // 进入队列
            colorQueue[queueTop].x = next_x, colorQueue[queueTop].y = next_y;
            queueTop ++; // 元素加一
        }
    }
    return color_cnt;
}

void bfsFilter(uint16 image[][IMAGE_W]) {
    // 将标签数组清零
    memset(visited, 0, sizeof visited);
    uint16 colorMaxCount = 0, lable = 1;
    for (uint16 r = 0; r < IMAGE_H; r ++) {
        for (uint16 c = 0; c < IMAGE_W; c ++) {
            if ((image[r][c] == RGB565_WHITE) && !visited[r][c]) {
                // 统计最大白块
                uint16 size = bfsToCountColor(image, r, c, RGB565_WHITE, lable);
                if (size > colorMaxCount) {
                    maxLable = lable;
                    colorMaxCount = size;
                }
                lable += 1;
            }
        }
    }
    // 将除了最大白块其他变成黑色
    for (uint16 r = 0; r < IMAGE_H; r ++ ) {
        for (uint16 c = 0; c < IMAGE_W; c ++) {
            if (visited[r][c] != maxLable) {
                image[r][c] = RGB565_BLACK;
            }
        }
    }
}

uint16 combineColor(uint16 red, uint16 green, uint16 blue) {
    return ((red & 0x1F) << 11) | ((green & 0x3F) << 5) | (blue & 0x1F);
}

// 从RGB565像素中提取单个通道值
uint16 getRed(uint16 pixel) {
    return (pixel >> 11) & 0x1F;
}

uint16 getGreen(uint16 pixel) {
    return (pixel >> 5) & 0x3F;
}

uint16 getBlue(uint16 pixel) {
    return pixel & 0x1F;
}


void Pixle_Filter(uint16 image[][IMAGE_W]) {
    for (int nr = 1; nr < IMAGE_H - 1; nr ++) {
        for (int nc = 1; nc < IMAGE_W - 1; nc ++) {
            int white = 0, black = 0;
            for (int k = 0; k < 4; k ++) {
                int x = nr + dx[k], y = nc + dy[k];
                if (image[x][y] == RGB565_BLACK) {
                    black ++;
                }
                else {
                    white ++;
                }
            }
            if (image[nr][nc] == RGB565_BLACK && white >= 3) {
                image[nr][nc] = RGB565_WHITE;
            }
            else if (image[nr][nc] == RGB565_WHITE && black >= 4) {
                image[nr][nc] = RGB565_BLACK;
            }
        }
    }
}

void Compressimage(void) {
    int i, j, row, line;
    const float div_h = SCC8660_H / IMAGE_H, div_w = SCC8660_W / IMAGE_W;
    for (i = 0; i < IMAGE_H; i ++)
    {
        row = i * div_h + 0.5; // +0.5是为了四舍五入
        for (j = 0; j < IMAGE_W; j ++)
        {
            line = j * div_w + 0.5;
            Image_Used[i][j] = scc8660_image[row][line];
            R_average += getRed(Image_Used[i][j]); // (Image_Used[i][j] >> 11) & 0x3f;
            G_average += getGreen(Image_Used[i][j]); // (Image_Used[i][j] >> 5) & 0x3f;
            B_average += getBlue(Image_Used[i][j]); // Image_Used[i][j] & 0x1f;
        }
    }
    R_average /= IMAGE_H * IMAGE_W;
    G_average /= IMAGE_H * IMAGE_W;
    B_average /= IMAGE_H * IMAGE_W;
}

void Equalizehist(uint16 Src[][IMAGE_W], uint16 Dst[][IMAGE_W]) {

}

void BuildTable(float fPrecompensation)
{
    int i;
    float f;
    for(i = 0; i < 32; i ++)
    {
        f = (i + 0.5F) / 32;//归一化
        f = (float)pow(f, fPrecompensation);
        g_GammaLUT_R[i] = (uint16) (f * 32 - 0.5F);//反归一化
    }
    for(i = 0; i < 64; i ++)
    {
        f = (i + 0.5F) / 64;//归一化
        f = (float)pow(f, fPrecompensation);
        g_GammaLUT_G[i] = (uint16) (f * 64 - 0.5F);//反归一化
    }
    for(i = 0; i < 32; i ++)
    {
        f = (i + 0.5F) / 32;//归一化
        f = (float)pow(f, fPrecompensation);
        g_GammaLUT_B[i] = (uint16) (f * 32 - 0.5F);//反归一化
    }
}

void GammaCorrectiom(uint16 src[], int iWidth, int iHeight, float fGamma, uint16 Dst[])
{
    int iCols, iRows;
    BuildTable(1 / fGamma); //gamma校正查找表初始化
    //对图像的每个像素进行查找表矫正
    color_channels rgb;
    for(iRows = 0; iRows < iHeight; iRows++)
    {
        for(iCols = 0; iCols < iWidth; iCols++)
        {
            GetRgb565(src[iRows * iWidth + iCols], &rgb);
            rgb.first = g_GammaLUT_R[rgb.first], rgb.second = g_GammaLUT_G[rgb.second], rgb.third = g_GammaLUT_B[rgb.third];
            Dst[iRows * iWidth + iCols] = combineColor(rgb.first, rgb.second, rgb.third);
        }
    }
}

void AutoBuildTable()
{
    int i;
    float f;
    float fPrecompensation_R = log10f(CENTER_BRIGHTNESS) / log10f(R_average / 31);//gamma = -0.3/log10(X)
    float fPrecompensation_G = log10f(CENTER_BRIGHTNESS) / log10f(G_average / 63);//gamma = -0.3/log10(X)
    float fPrecompensation_B = log10f(CENTER_BRIGHTNESS) / log10f(B_average / 31);//gamma = -0.3/log10(X)
//    float fPrecompensation = (fPrecompensation_R + fPrecompensation_G + fPrecompensation_B) / 3;
//    fPrecompensation_R = fPrecompensation_G = fPrecompensation_B = fPrecompensation;
    for(i = 0; i < 32; i ++)
    {
        f = (i + 0.5F) / 32;//归一化
        f = (float)pow(f, fPrecompensation_R);
        g_GammaLUT_R[i] = (uint16) (f * 32 - 0.5F);//反归一化
    }
    for(i = 0; i < 64; i ++)
    {
        f = (i + 0.5F) / 64;//归一化
        f = (float)pow(f, fPrecompensation_G);
        g_GammaLUT_G[i] = (uint16) (f * 64 - 0.5F);//反归一化
    }
    for(i = 0; i < 32; i ++)
    {
        f = (i + 0.5F) / 32;//归一化
        f = (float)pow(f, fPrecompensation_B);
        g_GammaLUT_B[i] = (uint16) (f * 32 - 0.5F);//反归一化
    }
}

void AutoGammaCorrectiom(uint16 src[], int iWidth, int iHeight, uint16 Dst[])
{
    int iCols, iRows;
    AutoBuildTable(); //gamma校正查找表初始化
    //对图像的每个像素进行查找表矫正
    color_channels rgb;
    for(iRows = 0; iRows < iHeight; iRows++)
    {
        for(iCols = 0; iCols < iWidth; iCols++)
        {
            GetRgb565(src[iRows * iWidth + iCols], &rgb);
            rgb.first = g_GammaLUT_R[rgb.first], rgb.second = g_GammaLUT_G[rgb.second], rgb.third = g_GammaLUT_B[rgb.third];
            Dst[iRows * iWidth + iCols] = combineColor(rgb.first, rgb.second, rgb.third);
        }
    }
}

uint16 log_transform(float color, float color_max) {
    float e = 2.718281828459;
    const float c = 1.0;
    float transformed_color = c * log2f(1 + 1.0 * color / color_max) / log2f(e);
    return transformed_color * color_max;
}

void Image_LogTransform(uint16 src[], int iWidth, int iHeight, uint16 Dst[]) {
    int iCols, iRows;
    color_channels hsl, rgb;
    for(iRows = 0; iRows < iHeight; iRows++)
    {
        for(iCols = 0; iCols < iWidth; iCols++)
        {
            GetRgb565(src[iRows * iWidth + iCols], &rgb);
            Rgb565ToHsl(&rgb, &hsl);
            log_transform(hsl.third, 1.0);
            rgb = HslToRgb565(&hsl);
            Dst[iRows * iWidth + iCols] = combineColor(rgb.first, rgb.second, rgb.third);
        }
    }
}

void ImagePerspective_Init(void) {
    static uint16_t BlackColor = 0; // 定义一个静态黑色像素值
    double change_un_Mat[3][3] = { // 定义透视变换矩阵
        { -0.01609759704190238, 0.01932561893613478, -2.040617594981866 },
        { 0.0004352209945470896, -0.000367865364438621, -0.7035606436969671 },
        { 1.115951268069474e-005, 0.0001970185393508392, -0.03104642853440032 }
    };

    for (int i = 0; i < RESULT_COL; i++) {
        for (int j = 0; j < RESULT_ROW; j++) {
            // 计算透视变换后的坐标
            int local_x = (int)((change_un_Mat[0][0] * i + change_un_Mat[0][1] * j + change_un_Mat[0][2]) /
                                (change_un_Mat[2][0] * i + change_un_Mat[2][1] * j + change_un_Mat[2][2]));
            int local_y = (int)((change_un_Mat[1][0] * i + change_un_Mat[1][1] * j + change_un_Mat[1][2]) /
                                (change_un_Mat[2][0] * i + change_un_Mat[2][1] * j + change_un_Mat[2][2]));

            // 如果变换后的坐标在原图像范围内
            if (local_x >= 0 && local_y >= 0 && local_y < USED_ROW && local_x < USED_COL) {
                PerImg_ip[j][i] = &PER_IMG[local_y][local_x]; // 赋值对应像素
            } else {
                PerImg_ip[j][i] = &BlackColor; // 否则赋值为黑色
            }
        }
    }
}

// 获取二值化图像的四个拐角坐标，方便形状的判断
void get_corner_points(uint16 image[][IMAGE_W], point pts[4]) {
    static uint16 inf = IMAGE_W + 2;
    point points[4] = { {inf, inf}, {0, inf}, {0, 0}, {inf, 0} };
    for (uint16 i = 1; i < IMAGE_H; i ++) {
        for (uint16 j = 1; j < IMAGE_W; j ++) {
            if (image[i][j] == RGB565_WHITE) {
                // 左上
                if (points[0].x + points[0].y > i + j) {
                    points[0].x = i, points[0].y = j;
                }
                // 左下
                if (points[1].x - points[1].y < i - j) {
                    points[1].x = i, points[1].y = j;
                }
                // 右下
                if (points[2].x + points[2].y < i + j) {
                    points[2].x = i, points[2].y = j;
                }
                // 右上
                if (points[3].x - points[3].y > i - j) {
                    points[3].x = i, points[3].y = j;
                }
            }
        }
    }
    for (int i = 0; i < 4; i ++) {
        pts[i].x = points[i].x;
        pts[i].y = points[i].y;
    }
}

// 计算两点之间的距离
double distance(point a, point b) {
    return sqrt(pow(b.x - a.x, 2) + pow(b.y - a.y, 2));
}

// 计算角度
double angle(point a, point b, point c) {
    double ab = distance(a, b);
    double bc = distance(b, c);
    double ac = distance(a, c);
    return acos((ab * ab + bc * bc - ac * ac) / (2 * ab * bc)) * 180.0 / PI;
}

// 判断是否为矩形
int isRectangle(point pts[4]) {
    double angle1 = angle(pts[0], pts[1], pts[2]);
    double angle2 = angle(pts[1], pts[2], pts[3]);
    double angle3 = angle(pts[2], pts[3], pts[0]);
    double angle4 = angle(pts[3], pts[0], pts[1]);

    if (fabs(angle1 - 90) < 10 && fabs(angle2 - 90) < 10 && fabs(angle3 - 90) < 10 && fabs(angle4 - 90) < 10) {
        return 1;
    }
    return 0;
}

//判断是否为方形
int isSquare(point pts[4]) {
    if (isRectangle(pts)) {
        double d1 = distance(pts[0], pts[1]);
        double d2 = distance(pts[1], pts[2]);
        double d3 = distance(pts[2], pts[3]);
        double d4 = distance(pts[3], pts[0]);

        if (fabs(d1 - d2) < 10 && fabs(d2 - d3) < 10 && fabs(d3 - d4) < 10 && fabs(d4 - d1) < 10) {
            return 1;
        }
    }
    return 0;
}

void get_point(float x, float y, point* pt, point* bias, START_POS pos) {
    if (pos == RIGHT_BOTTOM) {
        pt->y = tft180_height_max - 1 - y   / (region_length * NAVIGATION_PER_MILES) * tft180_height_max - bias->y;
        pt->x = tft180_width_max - 1 - (-x) / (region_width * NAVIGATION_PER_MILES) * tft180_width_max - bias->x;
    }
    else if (pos == LEFT_BOTTOM) {
        pt->y = tft180_height_max - 1 - y / (region_length * NAVIGATION_PER_MILES) * tft180_height_max - bias->y;
        pt->x = x / (region_width * NAVIGATION_PER_MILES) * tft180_width_max + bias->x;
    }


    pt->x = fclip(pt->x, START_POINT_X_MIN, START_POINT_X_MAX - 1);
    pt->y = fclip(pt->y, START_POINT_Y_MIN, START_POINT_Y_MAX - 1);
}

uint8 shapeJudge( point pts[4]) {
    float d1 = distance(pts[0], pts[1]);
    float d2 = distance(pts[1], pts[2]);
    float d3 = distance(pts[2], pts[3]);
    float d4 = distance(pts[3], pts[0]);

    float d1_avr = (d1 + d3) / 2, d2_avr = (d2 + d4) / 2;
    if (d1_avr / d2_avr >= 3 || d1_avr / d2_avr <= 0.33) {
        return 0;
    }
    return 1;
}

void warpPerspective(uint16 warped[][IMAGE_W]) {
    for(int i = 0;i < RESULT_ROW; i ++) {
        for(int j = 0;j < RESULT_COL;j ++) {
            warped[i][j] = ImageUsed[i][j];
        }
    }
}

// 腐蚀函数
void Erosion(uint16 image[IMAGE_H][IMAGE_W], uint16 eroded_image[IMAGE_H][IMAGE_W]) {

}

// 膨胀函数
void Dilation(uint16 image[IMAGE_H][IMAGE_W]) {

}

#pragma section all restore
