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
    bool is_cross;         // 十字路口标志 ok
    bool is_island;        // 环岛标志 ok
    bool is_ramp;          // 坡道标志
    bool is_zebra;         // 斑马线/起跑线标志
    bool is_straight;      // 直道标志 ok
    bool is_barrier;       // 障碍物标志
    bool allow_change_special_barrier;  //允许修改标志 
    bool is_special_barrier;   
    BypassDir_e special_barrier_dir;

    // 视野前瞻与安全参数
    int longest_white_len = TRACK_H; // 最长白列长度 (视野深度)
    int longest_white_x;   // 最长白列对应的 X 坐标 (寻线锚点)
    int white_pixel_count[TRACK_W];
    
    // 停车控制
    int stop_flag; //0：停车 1：预备发车 2：发车        
} TrackState_t;


extern TrackState_t track_state; // 赛道状态结构体

#endif

//定义TCP服务器地址
#define SERVER_IP "192.168.1.132" 
#define PORT_VOFA 1347
#define PORT_SEEKFREE 8086

#define MAX_DUTY  (90 )   // 最大 MAX_DUTY% 占空比

//定义巡线摄像头参数
#define TRACK_W 160
#define TRACK_H 80

//定义识别代码识别的参数，摄像头的在aiuvc里，这里逆时针翻转180度
#define AI_W 480
#define AI_H 640