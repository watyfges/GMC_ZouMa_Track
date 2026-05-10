#ifndef _CROSS_H_
#define _CROSS_H_

#include "head.h"

// 结构体：用来存放找到的角点坐标
typedef struct {
    int y;
    int x;
    bool found;
} CornerPoint_t;

bool Cross_Detect(void);

#endif