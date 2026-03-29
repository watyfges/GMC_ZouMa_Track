#include "motor.h"

// 右侧电机 (Motor 1)
#define MOTOR_RIGHT_PWM_PATH    ZF_PWM_MOTOR_2
#define MOTOR_RIGHT_DIR_PATH    ZF_GPIO_MOTOR_2

// 左侧电机 (Motor 2)
#define MOTOR_LEFT_PWM_PATH     ZF_PWM_MOTOR_1
#define MOTOR_LEFT_DIR_PATH     ZF_GPIO_MOTOR_1


// 定义正反转的高低电平 (如果发现小车往前走时轮子往后转，直接在这里把 1 和 0 对调即可)
#define MOTOR_DIR_FORWARD       1   // 前进时的方向引脚电平
#define MOTOR_DIR_REVERSE       0   // 后退时的方向引脚电平

// 占空比计算比例参数 (将 0~100 的输入百分比映射到硬件真实占空比)
#define DUTY_SCALE_FACTOR       100.0f

// 实例化GPIO方向控制对象 (O_RDWR 表示可读可写)
zf_driver_gpio  motor_right_dir(MOTOR_RIGHT_DIR_PATH, O_RDWR);
zf_driver_gpio  motor_left_dir(MOTOR_LEFT_DIR_PATH, O_RDWR);

// 实例化PWM调速控制对象
zf_driver_pwm   motor_right_pwm(MOTOR_RIGHT_PWM_PATH);
zf_driver_pwm   motor_left_pwm(MOTOR_LEFT_PWM_PATH);

// 定义用于存放底层PWM参数(如最大占空比)的结构体
struct pwm_info motor_right_pwm_info;
struct pwm_info motor_left_pwm_info;

// 获取设备树中的最大占空比 (必须在 get_dev_info 之后调用才有效)
#define MOTOR_RIGHT_PWM_MAX     (motor_right_pwm_info.duty_max)
#define MOTOR_LEFT_PWM_MAX      (motor_left_pwm_info.duty_max)


// ==========================================
// 2. 函数实现区域
// ==========================================

/**
 * @brief 电机模块初始化
 * @note  必须在 main 函数开头调用，用于获取硬件设备树配置
 */
void Motor_init(void)
{   
    // 从底层读取 PWM 参数，存入我们定义的结构体中
    motor_right_pwm.get_dev_info(&motor_right_pwm_info);
    motor_left_pwm.get_dev_info(&motor_left_pwm_info);
    
    // 初始化时确保电机处于停止状态
    motor_right_pwm.set_duty(0);
    motor_left_pwm.set_duty(0);
    printf("Motor Init Success");
}

/**
 * @brief 设置右侧电机占空比
 * @param duty 占空比数值，正数为前进，负数为后退
 */
void Set_Right_Pwm(float duty)
{
    if(duty < 0) {
        motor_right_dir.set_level(MOTOR_DIR_REVERSE);
        motor_right_pwm.set_duty(-duty * (MOTOR_RIGHT_PWM_MAX / DUTY_SCALE_FACTOR));
    } else {
        motor_right_dir.set_level(MOTOR_DIR_FORWARD);
        motor_right_pwm.set_duty(duty * (MOTOR_RIGHT_PWM_MAX / DUTY_SCALE_FACTOR));
    }
}

/**
 * @brief 设置左侧电机占空比
 * @param duty 占空比数值，正数为前进，负数为后退
 */
void Set_Left_Pwm(float duty)
{
    if(duty < 0) {
        motor_left_dir.set_level(MOTOR_DIR_REVERSE);
        motor_left_pwm.set_duty(-duty * (MOTOR_LEFT_PWM_MAX / DUTY_SCALE_FACTOR));
    } else {
        motor_left_dir.set_level(MOTOR_DIR_FORWARD);
        motor_left_pwm.set_duty(duty * (MOTOR_LEFT_PWM_MAX / DUTY_SCALE_FACTOR));
    }
}

/**
 * @brief 异常退出或程序结束时的清理操作
 */
void Motor_Cleanup(void)
{
    printf("执行电机模块清理：切断电机动力输出。\n");
    // 将占空比强制设为0，防止飞车
    motor_right_pwm.set_duty(0);   
    motor_left_pwm.set_duty(0);    
}