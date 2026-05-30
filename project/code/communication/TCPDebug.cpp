#include "TCPDebug.h"

uint8 current_camera_view = VIEW_TRACK_CAMERA; // 默认显示巡线画面
uint8 allow_send_image_flag = 0; //允许发送的标志位
uint16 ai_image_copy[AI_H][AI_W];


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
    uint8 rx_buf[65]; // 多留一位给字符串结束符 '\0'
    
    // 读取数据，传入 sizeof(rx_buf)-1 防止填满后无法放置结束符
    uint32 len = vofa_tcp.read_data(rx_buf, sizeof(rx_buf) - 1);
    
    if (len > 0) {
        rx_buf[len] = '\0'; // 强制在末尾添加字符串结束符，使其变成合法 C 字符串
        char *cmd_str = (char *)rx_buf;
        char *ptr = NULL;   // 用于定位字符串的指针

        // 提示音
        beep_on();
        system_delay_ms(50); // 稍微缩短一点延时，防止阻塞主循环太久
        beep_off();

        // --------------------------------------------------
        // 1. 处理 start / stop 控制指令
        // --------------------------------------------------
        if (strstr(cmd_str, "start") != NULL) {
            track_state.stop_flag = 2;
            printf("Command: start, stop_flag set to 2\r\n");
        } 
        else if (strstr(cmd_str, "stop") != NULL) {
            track_state.stop_flag = 0;
            printf("Command: stop, stop_flag set to 0\r\n");
        }

        // --------------------------------------------------
        // 2. 解析 左电机 PID 参数 (例如 "LDJKp3.0")
        // --------------------------------------------------
        if ((ptr = strstr(cmd_str, "LDJKp")) != NULL) {
            Left_Motor_Kp = atof(ptr + 5); // "+5" 是跳过 "LDJKp" 前 5 个字符，直接解析后面的数字
            printf("Set Left Kp: %f\r\n", Left_Motor_Kp);
        }
        if ((ptr = strstr(cmd_str, "LDJKi")) != NULL) {
            Left_Motor_Ki = atof(ptr + 5);
            printf("Set Left Ki: %f\r\n", Left_Motor_Ki);
        }
        if ((ptr = strstr(cmd_str, "LDJKd")) != NULL) {
            Left_Motor_Kd = atof(ptr + 5);
            printf("Set Left Kd: %f\r\n", Left_Motor_Kd);
        }

        // --------------------------------------------------
        // 3. 解析 右电机 PID 参数 (例如 "RDJKp3.0")
        // --------------------------------------------------
        if ((ptr = strstr(cmd_str, "RDJKp")) != NULL) {
            Right_Motor_Kp = atof(ptr + 5);
            printf("Set Right Kp: %f\r\n", Right_Motor_Kp);
        }
        if ((ptr = strstr(cmd_str, "RDJKi")) != NULL) {
            Right_Motor_Ki = atof(ptr + 5);
            printf("Set Right Ki: %f\r\n", Right_Motor_Ki);
        }
        if ((ptr = strstr(cmd_str, "RDJKd")) != NULL) {
            Right_Motor_Kd = atof(ptr + 5);
            printf("Set Right Kd: %f\r\n", Right_Motor_Kd);
        }

        //解析PID参数
        if ((ptr = strstr(cmd_str, "dis")) != NULL) {
            Dis = atof(ptr + 3);
            printf("Set dis: %d\r\n", Dis);
        }
        if ((ptr = strstr(cmd_str, "kp1")) != NULL) {
            kp1 = atof(ptr + 3);
            printf("Set Kp1: %f\r\n", kp1);
        }
        if ((ptr = strstr(cmd_str, "kp2")) != NULL) {
            kp2 = atof(ptr + 3);
            printf("Set Kp2: %f\r\n", kp2);
        }
        if ((ptr = strstr(cmd_str, "kd")) != NULL) {
            kd = atof(ptr + 2);
            printf("Set Kd: %f\r\n", kd);
        }
        if ((ptr = strstr(cmd_str, "speed")) != NULL) {
            Speed = atof(ptr + 5);
            printf("Set Speed: %f\r\n", Speed);
        }
        if ((ptr = strstr(cmd_str, "kz")) != NULL) {
            kz = atof(ptr + 2);
            printf("Set kz: %f\r\n", kz);
        }
        //测试绕标志牌
        if ((ptr = strstr(cmd_str, "DIRL")) != NULL) {
            track_state.special_barrier_dir = DIR_LEFT;
            track_state.is_special_barrier = true;

            if(track_state.is_special_barrier){
            track_state.allow_change_special_barrier=false;
            }
            printf("LEFT\r\n");
        }
        if ((ptr = strstr(cmd_str, "DIRR")) != NULL) {
            track_state.special_barrier_dir = DIR_RIGHT;
            track_state.is_special_barrier = true;

            if(track_state.is_special_barrier){
            track_state.allow_change_special_barrier=false;
            }
            printf("RIGHT\r\n");
        }

        // 4. 'T' 和 'A' 摄像头切换
        if (len <= 3) { // 考虑到串口可能带 \r \n，限制长度
            if (cmd_str[0] == 'T' || cmd_str[0] == 't') {
                current_camera_view = VIEW_TRACK_CAMERA;
                printf("Switched to Track Camera View.\r\n");
            } else if (cmd_str[0] == 'A' || cmd_str[0] == 'a') {
                current_camera_view = VIEW_AI_CAMERA;
                printf("Switched to AI Camera View.\r\n");
            }
        }
    }
#endif
}

// ==========================================================
// 逐飞助手传图任务 (放在主循环)
// ==========================================================
static void Seekfree_Send_Image_Task() {
#if CURRENT_DEBUG_MODE == DEBUG_MODE_BOTH
    // 只有允许发送标志位置 1 时，才发送一帧图像（防止撑爆TCP缓冲区）
    if (allow_send_image_flag == 0) return;
    allow_send_image_flag = 0; // 清除标志位，等待下一个周期
    if (current_camera_view == VIEW_TRACK_CAMERA) {
        // 1. 巡线模式 (带边界的灰度图)
        seekfree_assistant_camera_information_config(
            SEEKFREE_ASSISTANT_MT9V03X, 
            img_bin[0], // 请确保这里是160x120的二值化/灰度首地址
            //img_gray[0],
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
        for (uint32_t y = 0; y < AI_H; y++)
        {
            for (uint32_t x = 0; x < AI_W; x++)
            {
                uint16_t pixel = img_ai[y][x];

                 // 进行大小端字节交换，并存入备份数组
                ai_image_copy[y][x] = ((pixel & 0x00FF) << 8) |
                              ((pixel & 0xFF00) >> 8);
            }
        }
        seekfree_assistant_camera_information_config(
            SEEKFREE_ASSISTANT_RGB565, 
            ai_image_copy[0], 
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

void tcp_pit_callback() {
    // 增加一个静态计数器限制发图帧率
    static int img_timer_cnt = 0;
    img_timer_cnt++;
    if (img_timer_cnt >= 10) { // 10ms * 10 = 100ms (即限制为10FPS传图)
        img_timer_cnt = 0;
        allow_send_image_flag = 1;
    }
}