#ifndef __GLOBAL_STATE_H__
#define __GLOBAL_STATE_H__

#include "head.h"

// 定义特殊避障的绕行方向
typedef enum {
    DIR_NONE = 0,   // 无特殊障碍
    DIR_LEFT,       // 从左侧绕过
    DIR_RIGHT,      // 从右侧绕过
    DIR_STRAIGHT    // 垂直/直接穿过
} BypassDir_e;

// 赛道状态结构体
typedef struct {
    bool is_cross;         // 十字路口标志
    bool is_island;        // 环岛标志
    bool is_ramp;          // 坡道标志
    bool is_zebra;         // 斑马线/起跑线标志
    bool is_straight;      // 直道标志
    bool is_barrier;       // 障碍物标志
    bool is_special_barrier;   
    BypassDir_e special_barrier_dir;

    // 视野前瞻与安全参数
    int longest_white_len; // 最长白列长度 (视野深度)
    int longest_white_x;   // 最长白列对应的 X 坐标 (寻线锚点)
    
    // 停车控制
    int stop_flag;         
} TrackState_t;


extern TrackState_t track_state; // 赛道状态结构体

#endif