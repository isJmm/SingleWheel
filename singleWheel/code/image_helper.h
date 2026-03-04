#ifndef _image_helper_h_
#define _image_helper_h_

#include "head.h"

typedef struct {
    float x, y;
}point;

#define CENTER_BRIGHTNESS 0.5 // gamma自适应算法的中心亮度，会向这个亮度靠近
#define STRUCTURING_ELEMENT_SIZE 3 // 卷积的矩阵大小
#define COLOR_PROPORTION    5 // 颜色比例

typedef enum {
        LEFT_BOTTOM,
        RIGHT_BOTTOM,
}START_POS;


extern point corner_points[4];
extern START_POS start_pos;

void Pixle_Filter(uint16 image[][IMAGE_W]);
void GammaCorrectiom(uint16 src[], int iWidth, int iHeight, float fGamma, uint16 Dst[]);
void AutoGammaCorrectiom(uint16 src[], int iWidth, int iHeight, uint16 Dst[]);
void Image_LogTransform(uint16 src[], int iWidth, int iHeight, uint16 Dst[]);
void Erosion(uint16 image[IMAGE_H][IMAGE_W], uint16 eroded_image[IMAGE_H][IMAGE_W]);
void Dilation(uint16 image[IMAGE_H][IMAGE_W]);
void Compressimage(void);
void bfsFilter(uint16 image[][IMAGE_W]);
uint16 getRed(uint16 pixel);
uint16 getGreen(uint16 pixel);
uint16 getBlue(uint16 pixel);
uint16 combineColor(uint16 red, uint16 green, uint16 blue);
void warpPerspective(uint16 warped[][IMAGE_W]);
void ImagePerspective_Init(void);
void get_corner_points(uint16 image[][IMAGE_W], point pts[4]);
int isSquare(point pts[4]);
double distance(point a, point b);
uint8 shapeJudge( point pts[4]);
void get_point(float x, float y, point* pt, point* bias, START_POS pos);


#endif /* CODE_IMAGE_ALGORITHM_H_ */

