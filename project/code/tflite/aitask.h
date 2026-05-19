#ifndef AITASK_H
#define AITASK_H

#include "head.h"

// 图像缓存区声明
extern uint16_t img_ai[AI_H][AI_W]; // 原始灰度图

// 图像处理主程序
int ai_init(void);
void ai_extract(void);

#endif
