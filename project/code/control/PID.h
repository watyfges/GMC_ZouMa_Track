/* PID.h */
#ifndef PID_H
#define PID_H

#include "head.h"

typedef struct {
    float Kp1;
    float Kp2;
    float sigma;
    float Kd;
    float Kz;
    float prev_error;
    float max_output;
    float min_output;
    float dead_zone;
} Dynamic_PD;

typedef struct {
    float P;              //Kp系数
    float I;              //Ki系数
    float D;              //Kd系数
    float max_output;      //输出最大值，用于限幅	
	
    float LastError;     //前一次误差    
    float PrevError;     //前两次误差
}IncrementalPID;

void Dynamic_PD_Init(Dynamic_PD *pd, float Kp1, float Kp2, 
                     float sigma, float Kd, float Kz, float max_out, float min_out, 
                     float dead_zone);

float Dynamic_PD_Compute(Dynamic_PD *pd, float setpoint, float current);

void PID_Init(IncrementalPID *pid, float Kp, float Ki, float Kd, 
              float max_out);

float IncPIDCal(IncrementalPID *pid, int32_t AimValue, int32_t NowValue);

#endif