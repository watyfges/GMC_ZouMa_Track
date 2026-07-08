/* PID.h */
#ifndef PID_H
#define PID_H

#include "head.h"



void outer_dir_pid(float error);
void inner_dir_pid(float gyro_z);
void l_speed_pid(float aim_speed, float now_speed);
void r_speed_pid(float aim_speed, float now_speed);


#endif