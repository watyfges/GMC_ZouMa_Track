/* PID.c */
#include "PID.h"

float kd_2;
float absolute_error;

extern float GZ;
extern int Longest_White_Column;//最长白色列宽度
extern int16 encoder_left;//左侧编码器值
extern int16 encoder_right;//右侧编码器值
extern float V_Left, V_Right;//左侧和右侧电机速度

/****************** 二次PD控制器 ******************/
void Dynamic_PD_Init(Dynamic_PD *pd, float Kp1, float Kp2, 
                     float sigma, float Kd, float Kz, float max_out, float min_out, 
                     float dead_zone) {
    pd->Kp1 = Kp1;//一次项比例系数
    pd->Kp2 = Kp2;//二次项比例系数
    pd->sigma = sigma;//1 正向  -1 反向
    pd->Kd = Kd;//微分项比例系数
    pd->Kz = Kz;//积分项比例系数
    pd->prev_error = 0.0f;//上次误差
    pd->max_output = max_out;//最大输出
    pd->min_output = min_out;//最小输出
    pd->dead_zone = dead_zone;//死区范围
}

// float Dynamic_PD_Compute(Dynamic_PD *pd, float setpoint, float current) {
//     float error = pd->sigma * current;
//     absolute_error = fabsf(error);
//     // 死区处理（减少微小误差的高频调整）[[8]]
//     if(absolute_error < pd->dead_zone) {
//         pd->prev_error = error;
//         return 0.0f;
//     }
//     //ips200_show_gray_image(0, 0, rgay_image, UVC_WIDTH, UVC_HEIGHT);
//     //ips200_show_float(  80, 16*8,   error,      8, 4);                  // 显示 float 数据 8bit 整数 4bit 小数 应当显示 13.1415 总共会有 14 个字符的显示占位 后面会有 5 个字符的空白占位
//     float F_kp = Fuzzy_P(error, error - pd->prev_error);

//     // 微分项计算（引入一阶惯性环节平滑噪声）[[8]]
//     float derivative = (error - pd->prev_error) * pd->Kd;
//     //tcp_send_test(error, error - pd->prev_error, error, pd->prev_error, error, error);
//     // PD输出公式（分离式控制结构）[[9]]


//     float output;

//     //output = (error * error * pd->Kp2 + pd->Kp1) * error + (error - pd->prev_error) * pd->Kd;//定模糊PID时使用
//     // pd->Kp2 = (absolute_error < 15) ? 0.0032 : 0.0011;
//     // pd->Kp1 = (absolute_error > 10.0) ? 8.9 : 12.5;
//     // pd->Kd = ((error - pd->prev_error)*error < 0) ? 28.0 : 18.0;
//     output = (error * error * pd->Kp2 + pd->Kp1) * error + (error - pd->prev_error) * pd->Kd + GZ * pd->Kz;
//     // 输出限幅（双极性限幅）[[5]]
//     output = fmaxf(pd->min_output, fminf(output, pd->max_output));
//     //tcp_send_test(V_Right, encoder_right, V_Left, encoder_left, error, output);
//     //printf("E:%f,EC:%f,output:%f\r\n",error , error - pd->prev_error , output);
//     //printf("EC:%f\r\n",error - pd->prev_error);
//     // 状态更新
//     pd->prev_error = error;
    
//     return output;
// }

float Dynamic_PD_Compute(Dynamic_PD *pd, float setpoint, float current) {
    float error = pd->sigma * current;
    absolute_error = fabsf(error);
    
    // 1. 死区处理
    if(absolute_error < pd->dead_zone) {
        pd->prev_error = error;
        return 0.0f;
    }

    float p_out = (error * absolute_error * pd->Kp2) + (error * pd->Kp1);

    // 3. 微分项低通滤波
    float current_derivative = error - pd->prev_error;
    static float filtered_derivative = 0.0f; 
    filtered_derivative = filtered_derivative * 0.7f + current_derivative * 0.3f;
    
    float d_out = filtered_derivative * pd->Kd;

    // 4. 总输出计算
    float output = p_out + d_out + GZ * pd->Kz;

    // 5. 输出限幅
    output = fmaxf(pd->min_output, fminf(output, pd->max_output));
    
    // 状态更新
    pd->prev_error = error;
    
    return output;
}

/****************** 增量式PID ******************/
void PID_Init(IncrementalPID *pid, float Kp, float Ki, float Kd, 
              float max_out) {

    pid->P = Kp;
    pid->I = Ki;
    pid->D = Kd;
    pid->max_output = max_out;

    pid->LastError = 0;
    pid->PrevError = 0;
}
 
/**********************************************************************************************************
*	函 数 名：IncPIDCal
*	功能说明：增量式PID计算
*	形    参：
*	返 回 值：
**********************************************************************************************************/ 
float IncPIDCal(IncrementalPID *pid, int32_t AimValue, int32_t NowValue)     
{
	int32_t iError;                          //当前误差值
	float Output;                           //控制输出增量值
	
	iError = AimValue - NowValue;            //目标值与当前值之差
 
	Output = (pid->P * (iError - pid->LastError))               //E[k]项
			+(pid->I * iError)       //E[k-1]项
			+(pid->D * (iError - 2 * pid->LastError + pid->PrevError));      //E[k-2]项
 
	pid->PrevError = pid->LastError;         //存储误差，用于下次计算
	pid->LastError = iError;
	
    Output /= 100;

    Output = Output > pid->max_output?pid->max_output:Output;  //控制输出上限幅
    Output = Output <-pid->max_output?-pid->max_output:Output; //控制输出下限幅
	
	return Output;                         //返回增量值
}