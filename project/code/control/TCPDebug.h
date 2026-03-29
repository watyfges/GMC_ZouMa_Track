#ifndef TCPDEBUG_H
#define TCPDEBUG_H

#include "head.h"

// ======================= 模式选择 =======================
#define DEBUG_MODE_NONE     1  // 模式1：不开启远程调参
#define DEBUG_MODE_VOFA     2  // 模式2：仅开启 Vofa 调参 (波形+发数据)
#define DEBUG_MODE_BOTH     3  // 模式3：同时开启 Vofa (波形/调参) 和 逐飞助手 (传图像)

// ★ 在这里修改当前的调试模式 ★
#define CURRENT_DEBUG_MODE  DEBUG_MODE_BOTH  

// ======================= 摄像头视图定义 =======================
#define VIEW_TRACK_CAMERA   0  // 巡线摄像头
#define VIEW_AI_CAMERA      1  // AI识别摄像头

extern uint8 current_camera_view;

// ======================= 外部变量声明 =======================
// (如果这些变量已经在 head.h 声明过，这里可省略，确保能访问到即可)
extern uint8_t img_bin[TRACK_H][TRACK_W];  // 巡线二值化图像 (根据你实际的名字修改)
extern uint8 L_boundary[];
extern uint8 M_boundary[];
extern uint8 R_boundary[];
extern uint16 img_ai[AI_H][AI_W]; // AI摄像头图像

// ======================= 接口声明 =======================
// 初始化调试模块 (传入服务器IP，以及两个不同的端口)
void Debug_Init(const char* ip, uint32 vofa_port, uint32 seekfree_port);

// 循环任务 (放在 while(1) 中调用)
void Debug_Task(void);

// Vofa 发送波形数据 (使用 JustFloat 协议，支持最多4个通道，按需增减)
void Vofa_Send_Waveform(float ch0, float ch1, float ch2, float ch3);

void tcp_pit_callback(void);

#endif // TCPDEBUG_H