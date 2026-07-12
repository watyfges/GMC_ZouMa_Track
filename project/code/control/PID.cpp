/* PID.c */
#include "PID.h"

extern float GZ;

//需要调的B玩意
extern float kp1,kp2,kd,kz; //kz，kp2没用
float inner_dir_kp = 1.0f;
float inner_dir_kd = 0.0f;


float outer_dir_error = 0.0f;
float outer_dir_last_error = 0.0f;
float outer_dir_last_out = 0.0f;

float inner_dir_error = 0.0f;
float inner_dir_last_error = 0.0f;
float inner_dir_last_out = 0.0f;


//勿动，内部用的电机变量
float l_speed_error = 0.0f;
float l_speed_last_error = 0.0f;
float l_speed_last_out = 0.0f;
float r_speed_error = 0.0f;
float r_speed_last_error = 0.0f;
float r_speed_last_out = 0.0f;

//勿动，两个环的输出
float outer_dir_out = 0.0f;
float inner_dir_out = 0.0f;

//勿动，给电机用的
float l_speed_ki = 0.0f;
float l_speed_kp = 0.0f;
float r_speed_ki = 0.0f;
float r_speed_kp = 0.0f;
float l_speed_out = 0.0f;
float r_speed_out = 0.0f;



void outer_dir_pid(float error)
{
    const float filter = 0.9f;

    outer_dir_last_out = outer_dir_out;
    outer_dir_last_error = outer_dir_error;
    outer_dir_error = error;

    outer_dir_out = kp1 * outer_dir_error
                  + kd * (outer_dir_error - outer_dir_last_error);
    outer_dir_out = outer_dir_out * filter + outer_dir_last_out * (1.0f - filter);
}

void inner_dir_pid(float gyro_z)
{
    const float filter = 0.9f;
    const float aim_gyro = outer_dir_out;

    inner_dir_last_out = inner_dir_out;
    inner_dir_last_error = inner_dir_error;
    inner_dir_error = aim_gyro + gyro_z;

    inner_dir_out = inner_dir_kp * inner_dir_error
                  + inner_dir_kd * (inner_dir_error - inner_dir_last_error);
    inner_dir_out = inner_dir_out * filter + inner_dir_last_out * (1.0f - filter);
}

void l_speed_pid(float aim_speed, float now_speed)
{
    const float filter = 0.9f;
    const float target_speed = aim_speed - inner_dir_out;

    l_speed_last_out = l_speed_out;
    l_speed_last_error = l_speed_error;
    l_speed_error = target_speed - now_speed;

    l_speed_out += l_speed_ki * l_speed_error
                 + l_speed_kp * (l_speed_error - l_speed_last_error);
    l_speed_out = l_speed_out * filter + l_speed_last_out * (1.0f - filter);
}

void r_speed_pid(float aim_speed, float now_speed)
{
    const float filter = 0.9f;
    const float target_speed =  aim_speed + inner_dir_out;

    r_speed_last_out = r_speed_out;
    r_speed_last_error = r_speed_error;
    r_speed_error = target_speed - now_speed;

    r_speed_out += r_speed_ki * r_speed_error
                 + r_speed_kp * (r_speed_error - r_speed_last_error);
    r_speed_out = r_speed_out * filter + r_speed_last_out * (1.0f - filter);
}