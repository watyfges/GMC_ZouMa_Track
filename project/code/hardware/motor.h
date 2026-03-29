#ifndef __MOTOR_H__
#define __MOTOR_H__

#include "zf_common_headfile.hpp"

void Motor_init(void);                  // 电机模块初始化
void Set_Right_Pwm(float duty);         // 设置右侧电机转速 (输入范围：-100 到 100)
void Set_Left_Pwm(float duty);          // 设置左侧电机转速 (输入范围：-100 到 100)
void Motor_Cleanup(void);               // 安全清理，关闭电机

#endif // __MOTOR_H__