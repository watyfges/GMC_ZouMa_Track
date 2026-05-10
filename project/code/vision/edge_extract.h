#ifndef EDGE_EXTRACT_H
#define EDGE_EXTRACT_H

#include "head.h"

// 大津法阈值
#define MIN_THRESHOLD 70

// 定义图像最底行（通常车头部分会有遮挡，不一定是 119，可按需修改）
#define LOWEST_ROW (TRACK_H - 1) 

// 赛道中位线
#define HALF_TRACK_WIDTH 40

// 图像缓存区声明
extern uint8_t img_gray[TRACK_H][TRACK_W]; // 原始灰度图
extern uint8_t img_bin[TRACK_H][TRACK_W];  // 二值化图像

// 边界线数组声明
extern uint8_t L_boundary[TRACK_H]; // 左边界
extern uint8_t R_boundary[TRACK_H]; // 右边界
extern uint8_t M_boundary[TRACK_H]; // 中线

// 图像处理主程序
void edge_extract(void);

#endif


