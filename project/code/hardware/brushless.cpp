#include "brushless.h"//建议3m/s以上速度再使用无刷电调


#define ESC_PATH        ZF_PWM_ESC_1
struct pwm_info esc_info;

zf_driver_pwm   esc_pwm(ESC_PATH);

void brushless_init()//初始化无刷电调
{
    // 获取PWM设备信息
    esc_pwm.get_dev_info(&esc_info);
}

void set_brushless_pwm(uint16 duty)//设置左侧无刷电调占空比
{
    // 计算无刷电调转速   （1ms - 2ms）/20ms * 10000（10000是PWM的满占空比时候的值）
    // 无刷电调转速 0%   为 500
    // 无刷电调转速 20%  为 600
    // 无刷电调转速 40%  为 700
    // 无刷电调转速 60%  为 800
    // 无刷电调转速 80%  为 900
    // 无刷电调转速 100% 为 1000

    // 修改duty的值，可以修改无刷电调转速
    esc_pwm.set_duty(duty);
}


void cleanup_brushless()//程序异常退出时的清理操作
{
    esc_pwm.set_duty(0);    
}