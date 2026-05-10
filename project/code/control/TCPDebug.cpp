#include "TCPDebug.h"

uint8 current_camera_view = VIEW_TRACK_CAMERA; // 默认显示巡线画面

// 根据模式按需实例化 TCP 客户端
#if CURRENT_DEBUG_MODE != DEBUG_MODE_NONE
zf_driver_tcp_client vofa_tcp;
#endif

#if CURRENT_DEBUG_MODE == DEBUG_MODE_BOTH
zf_driver_tcp_client seekfree_tcp;

// 逐飞助手 TCP 收发包装函数
uint32 seekfree_send_wrap(const uint8 *buf, uint32 len) {
    return seekfree_tcp.send_data(buf, len);
}
uint32 seekfree_read_wrap(uint8 *buf, uint32 len) {
    return seekfree_tcp.read_data(buf, len);
}
#endif

// Vofa+ JustFloat 协议的帧尾
const uint8 vofa_tail[4] = {0x00, 0x00, 0x80, 0x7f};

// ==========================================================
// 初始化
// ==========================================================
void Debug_Init(const char* ip, uint32 vofa_port, uint32 seekfree_port) {
#if CURRENT_DEBUG_MODE == DEBUG_MODE_NONE
    printf("Remote Debug Disabled.\r\n");
#endif

#if CURRENT_DEBUG_MODE == DEBUG_MODE_VOFA || CURRENT_DEBUG_MODE == DEBUG_MODE_BOTH
    if (vofa_tcp.init(ip, vofa_port) == 0) {
        printf("Vofa TCP Connected on port %d!\r\n", vofa_port);
    } else {
        printf("Vofa TCP Connect Failed!\r\n");
    }
#endif

#if CURRENT_DEBUG_MODE == DEBUG_MODE_BOTH
    if (seekfree_tcp.init(ip, seekfree_port) == 0) {
        printf("Seekfree TCP Connected on port %d!\r\n", seekfree_port);
        // 初始化逐飞助手接口
        seekfree_assistant_interface_init(seekfree_send_wrap, seekfree_read_wrap);
    } else {
        printf("Seekfree TCP Connect Failed!\r\n");
    }
#endif
}

// ==========================================================
// Vofa 波形发送 (使用 JustFloat 协议)
// ==========================================================
void Vofa_Send_Waveform(float ch0, float ch1, float ch2, float ch3) {
#if CURRENT_DEBUG_MODE == DEBUG_MODE_VOFA || CURRENT_DEBUG_MODE == DEBUG_MODE_BOTH
    float data[4] = {ch0, ch1, ch2, ch3};
    // 发送数据区
    vofa_tcp.send_data((uint8*)data, sizeof(data));
    // 发送帧尾
    vofa_tcp.send_data(vofa_tail, 4);
#endif
}

// ==========================================================
// 接收指令任务 (放在主循环)
// ==========================================================
static void Vofa_Receive_Task() {
#if CURRENT_DEBUG_MODE == DEBUG_MODE_VOFA || CURRENT_DEBUG_MODE == DEBUG_MODE_BOTH
    uint8 rx_buf[64];
    uint32 len = vofa_tcp.read_data(rx_buf, sizeof(rx_buf));
    beep_on();
    system_delay_ms(200);
    beep_off();
    if (len > 0) {
        // 解析 Vofa 发来的数据
        // 比如在 Vofa 文本发送框里发 'T' 切换巡线，发 'A' 切换AI
        for (uint32 i = 0; i < len; i++) {
            if (rx_buf[i] == 'T' || rx_buf[i] == 't') {
                current_camera_view = VIEW_TRACK_CAMERA;
                printf("Switched to Track Camera View.\r\n");
            } else if (rx_buf[i] == 'A' || rx_buf[i] == 'a') {
                current_camera_view = VIEW_AI_CAMERA;
                printf("Switched to AI Camera View.\r\n");
            }
            // 可以在这里扩展其他的调参协议解析逻辑...
        }
    }
#endif
}

// ==========================================================
// 逐飞助手传图任务 (放在主循环)
// ==========================================================
static void Seekfree_Send_Image_Task() {
#if CURRENT_DEBUG_MODE == DEBUG_MODE_BOTH
    if (current_camera_view == VIEW_TRACK_CAMERA) {
        // 1. 巡线模式 (带边界的灰度图)
        seekfree_assistant_camera_information_config(
            SEEKFREE_ASSISTANT_MT9V03X, 
            img_bin[0], // 请确保这里是160x120的二值化/灰度首地址
            TRACK_W, 
            TRACK_H
        );
        seekfree_assistant_camera_boundary_config(
            X_BOUNDARY, 
            TRACK_H, 
            L_boundary, M_boundary, R_boundary, 
            NULL, NULL, NULL
        );
        seekfree_assistant_camera_send();

    } else if (current_camera_view == VIEW_AI_CAMERA) {
        // 2. AI 摄像头模式
        // 假设 ai_img 是 uint16 的 RGB565 格式
        seekfree_assistant_camera_information_config(
            SEEKFREE_ASSISTANT_RGB565, 
            img_ai[0], 
            AI_W, 
            AI_H
        );
        seekfree_assistant_camera_boundary_config(
            NO_BOUNDARY, 0, NULL, NULL, NULL, NULL, NULL, NULL
        );
        seekfree_assistant_camera_send();
    }
#endif
}

// ==========================================================
// 综合调试循环任务
// ==========================================================
void Debug_Task() {
    Vofa_Receive_Task();
    Seekfree_Send_Image_Task();
}