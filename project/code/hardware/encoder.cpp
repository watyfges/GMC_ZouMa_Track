#include <encoder.h>

#define ENCODER_QUAD_1_PATH         ZF_ENCODER_QUAD_1
#define ENCODER_QUAD_2_PATH         ZF_ENCODER_QUAD_2

// 创建编码器对象，传入文件路径
zf_driver_encoder encoder_quad_1(ENCODER_QUAD_1_PATH);
zf_driver_encoder encoder_quad_2(ENCODER_QUAD_2_PATH);

float encoder_left = 0.0;//左侧编码器计数
float encoder_right = 0.0;//右侧编码器计数
int16 last_encoder_left=0.0, last_encoder_right=0.0;//上次左侧编码器计数，上次右侧编码器计数

void encoder_pit_callback() {
    // 定时器数值获取
    last_encoder_left = encoder_quad_2.get_count();
    last_encoder_right = -encoder_quad_1.get_count();
}

void clear_encoder_pit_callback() {
    // 定时器清零
    encoder_quad_1.clear_count();
    encoder_quad_2.clear_count();
}
