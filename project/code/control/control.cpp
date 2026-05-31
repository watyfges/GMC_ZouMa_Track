#include "control.h"//控制模块

#define PI 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679

using namespace std;

// 均值滤波
// 定义滑动窗口的大小，可以根据实际噪声情况调整（通常取 5~10）
#define FILTER_WINDOW 7 
// 定义静态数组作为历史数据缓冲区，保留在回调函数外部或作为静态变量
static float left_enc_buffer[FILTER_WINDOW] = {0};
static float right_enc_buffer[FILTER_WINDOW] = {0};
static uint8_t filter_index = 0;   // 当前写入数组的索引
static uint8_t filter_count = 0;   // 记录已存入的数据个数（处理刚启动时数据不满的情况

//电机PID定义
float Left_Motor_Kp = 7;
float Left_Motor_Ki = 0.09;
float Left_Motor_Kd = 0;
int Left_Motor_Maxduty = 300;
float Right_Motor_Kp = 7;
float Right_Motor_Ki = 0.09;
float Right_Motor_Kd = 0;
int Right_Motor_Maxduty = 300;

float V_Left, V_Right, Speed;//速度，左轮速度，右轮速度
float Right_Duty = 0, Left_Duty = 0; //左右电机占空比
int Dis = 0;//距离，范围0——74
//PID
IncrementalPID Left_Motor_PID, Right_Motor_PID;//左右电机增量式PID控制器
Dynamic_PD Servo_PD;    //二次PD控制器

//extern
extern float encoder_left, encoder_right;
extern int16 last_encoder_left, last_encoder_right;//编码器计数
extern uint8 M_boundary[TRACK_H];//中线边界数组
//extern float roll,pitch,yaw; //姿态角 **外部读取

//变量初始化
void control_signal_init() {
    Right_Motor_PID.LastError = 0; Right_Motor_PID.PrevError = 0;
    Left_Motor_PID.LastError = 0; Left_Motor_PID.PrevError = 0; Servo_PD.prev_error = 0;Right_Duty = 0;
    Left_Duty = 0;
}

//PID选择
void Choice_PID(int num) {
    switch(num){
        case 0://测试
                Speed = 200;
                Dis = 55;             // 前瞻距离 
                //Island_Time = 12;
                // 转向PD控制器 (输出左右轮速度差)
                Dynamic_PD_Init(&Servo_PD, 
                    1.0,    // Kp1 一次项
                    0.18,    // Kp2 二次项
                    1.0,    // 误差敏感度参数σ
                    1.0,   // Kd 微分系数
                    0,      // 陀螺仪参数Kz 
                    300,    // 输出上限 (最大允许的左右轮速度差)
                    -300,   // 输出下限
                    0       // 死区
                );
            break;
        default:
            break;
            }
} 

//初始化控制程序所有项目，变量，电机PID，选择PID参数
void control_init() {
    control_signal_init();
    // 电机速度闭环 PID (这部分决定了车轮跟调速指令有多紧)
    // PID_Init(&Right_Motor_PID, 19, 2.7, 0, 300);
    // PID_Init(&Left_Motor_PID,  19, 2.7, 0, 300);
    PID_Init(&Right_Motor_PID, Right_Motor_Kp, Right_Motor_Ki, Right_Motor_Kd, Right_Motor_Maxduty); //kp,ki,kd,maxduty
    PID_Init(&Left_Motor_PID, Left_Motor_Kp, Left_Motor_Ki, Left_Motor_Kd, Left_Motor_Maxduty);
    Choice_PID(0);
}

// 视觉误差计算
int last_error;
float Weighting[10] = {0.96, 0.92, 0.88, 0.83, 0.77, 0.71, 0.65, 0.59, 0.53, 0.47};
float Err_Sum() {
    float error = 0, weight_count = 0, now_mid = 0;
    if(Dis - 10 > LOWEST_ROW - track_state.longest_white_len) {
        for(int i = Dis; i > Dis - 10; i--) {
            now_mid += (float)M_boundary[i] * Weighting[Dis - i];
            weight_count += Weighting[Dis - i];
        }
    } else {
        int start_idx = LOWEST_ROW - track_state.longest_white_len + 9;
        if (start_idx > LOWEST_ROW) {
            start_idx = LOWEST_ROW;
        }
        int end_idx = start_idx - 10;
        if (end_idx < 0) {
            end_idx = -1; // 这样 i > end_idx 最小到 0
        }

        for(int i = start_idx; i > end_idx; i--) {
            now_mid += (float)M_boundary[i] * Weighting[start_idx - i];
            weight_count += Weighting[start_idx - i];
        }
    }

    now_mid = (now_mid + TRACK_W / 2.0f) / (weight_count + 1.0f);
    error = TRACK_W / 2.0f - now_mid;

    if(track_state.longest_white_len == 0) {
        error = last_error;
    }
    last_error = error;
    return error;
}


//void time_pit_callback() {}


//差速转向计算
void turn_pit_callback() {
    // 1. 获取视觉误差
    float error = Err_Sum();
    
    // 2. 计算速度补偿值 (Turn Offset)
    // 此时 Dynamic_PD 的输出代表：为了纠正偏差，左右轮需要产生多大的速度差
    float turn_offset = Dynamic_PD_Compute(&Servo_PD, 0, error);

    // 3. 差速分配逻辑
    V_Left  = Speed - turn_offset; 
    V_Right = Speed + turn_offset;
}

void motor_pit_callback() {
    // V_Right = 120.0f;
    // V_Left  = 120.0f; 
    // 读取编码器,写在encoder.cpp中

    // 编码器低通滤波
    encoder_left = last_encoder_left * 0.2f + encoder_left * 0.8f;
    encoder_right = last_encoder_right * 0.2f + encoder_right * 0.8f;

    // 编码器均值滤波
    // 1. 将最新数据存入缓冲区
    left_enc_buffer[filter_index] = encoder_left;
    right_enc_buffer[filter_index] = encoder_right;
    
    // 2. 更新索引和计数器
    filter_index++;
    if (filter_index >= FILTER_WINDOW) {
        filter_index = 0; // 索引回环，覆盖最老的数据
    }
    if (filter_count < FILTER_WINDOW) {
        filter_count++;   // 启动初期，记录当前缓冲区的有效数据量
    }

    // 3. 计算求和
    float sum_left = 0.0f;
    float sum_right = 0.0f;
    for (uint8_t i = 0; i < filter_count; i++) {
        sum_left += left_enc_buffer[i];
        sum_right += right_enc_buffer[i];
    }

    // 4. 计算均值，并覆盖原本的变量供给PID使用
    encoder_left = sum_left / filter_count;
    encoder_right = sum_right / filter_count;
    // ------------------- 滑动均值滤波结束 -------------------

    // 增量式 PID 计算PWM输出
    Right_Duty += IncPIDCal(&Right_Motor_PID, V_Right, (int32_t)encoder_right);
    Left_Duty  += IncPIDCal(&Left_Motor_PID,  V_Left,  (int32_t)encoder_left);

    // PWM 限幅保护
    if(fabs(Right_Duty) >= MAX_DUTY) Right_Duty = (Right_Duty / fabs(Right_Duty)) * MAX_DUTY;
    if(fabs(Left_Duty)  >= MAX_DUTY) Left_Duty  = (Left_Duty  / fabs(Left_Duty))  * MAX_DUTY;
    
    if(track_state.stop_flag==2){
        Set_Right_Pwm(Right_Duty);
        Set_Left_Pwm(Left_Duty);
    }else{
        Set_Right_Pwm(0);
        Set_Left_Pwm(0);
    }
    

    //printf("l_enc: %f, r_enc: %f, l_out: %f, r_out: %f\n", encoder_left, encoder_right,Left_Duty,Right_Duty);
    //printf("%f,%f,%f\n", V_Right, encoder_right, Right_Duty);
    //Vofa_Send_Waveform(V_Right, encoder_right, V_Left,encoder_left);
}

void cleanup() {
    Motor_Cleanup();
    beep_off();
}

// 中断总回调函数
// imu的写在imu.cpp里
void pit_callback() {
    encoder_pit_callback();
    turn_pit_callback(); 
    motor_pit_callback();
    clear_encoder_pit_callback();
    //time_pit_callback();
}
