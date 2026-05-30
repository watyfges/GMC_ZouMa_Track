#ifndef _ISLAND_H_
#define _ISLAND_H_

#include "head.h"

typedef enum {
    ISLAND_STATE_NONE = 0,          // 无环岛状态
    ISLAND_STATE_ENTRY_DETECTED ,   // 识别到入环拐点 1
    ISLAND_STATE_APPROACHING,       // 逼近入环尖角   2
    ISLAND_STATE_ENTERING,          // 正在切入环岛   3
    ISLAND_STATE_INSIDE,            // 环岛内圆周行驶 4
    ISLAND_STATE_EXIT_DETECTED,     // 识别到出环拐点 5
    ISLAND_STATE_EXIT_APPROACHING,  // 逼近出环尖角   6
    ISLAND_STATE_EXITING,           // 正在驶出环岛   7
    ISLAND_STATE_FINDEXIT_V,        // 找V  8
    ISLAND_STATE_ADJUSTING,         // 出环后姿态调整 9
    ISLAND_STATE_FINISHED           // 环岛结束复位   10
} IslandState_e;

typedef enum {
    NONE_ISLAND,
    LEFT_ISLAND,
    RIGHT_ISLAND
} IslandDir_e;

typedef struct {
    IslandState_e state;       // 当前环岛阶段 (0-9)
    IslandDir_e dir;            //当前环岛方向 L，N，R
} IslandController_t;

void Island_Detect();
bool Check_Straight_L();

#endif