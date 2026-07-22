#include "ramp.h"//坡道检测模块
//这里只使用了陀螺仪，如果想更准确可以使用距离传感器和陀螺仪结合使用

extern uint8_t L_boundary[TRACK_H], R_boundary[TRACK_H],M_boundary[TRACK_H];
extern float roll,pitch,yaw; //姿态角
int ramp_up=130,ramp_down=115,ramp_leave=130;
//extern int Longest_White_Column;//最长白列长度 

int Ramp_State = 0;//坡道状态机
extern volatile int16 dl1x_distance_raw;

// 最小二乘法计算斜率
float Ramp_calculateSlope(int begin, int end, uint8 border[TRACK_W]) {//计算斜率
    float sumX = 0, sumY = 0, sumXY = 0, sumXX = 0; // 求和变量
    static float lastValidSlope = 0; // 保留上次有效斜率

    for (int y = begin; y <= end; ++y) {//统计区间数据
        sumX += y;
        sumY += border[y];
        sumXY += y * border[y];
        sumXX += y * y;
    }

    int numPoints = end - begin + 1; // 计算点数
    float denominator = numPoints * sumXX - sumX * sumX;//计算分母

    // 防止除以零
    if (fabs(denominator) < 1e-5) return lastValidSlope;

    float slope = (numPoints * sumXY - sumX * sumY) / denominator;//计算斜率
    lastValidSlope = slope; // 更新保留值
    return slope; // 返回斜率
}

bool Ramp_check()
{     
    
    if(dl1x_distance_raw > 115){
        return 0;
    }
    return 1;
    

}


void Ramp_Detect()//坡道检测函数
{   
    if(track_state.is_ramp==false){//无坡道状态
        if(Ramp_check()){//检测到坡道
            Ramp_State = 1;//进入坡道状态机初始状态
            track_state.is_ramp = true;//置坡道标志位
        }
    }
    switch(Ramp_State)
    {
        case 1:
            if(dl1x_distance_raw > 130)//容错区间，需要测距离，上坡道时
            {
                Ramp_State = 2;
            }
            break;
        case 2:
            if(dl1x_distance_raw < 115)//需要测距离，下坡道时
            {
                Ramp_State = 3;
            }
            break;
        case 3:
            if(dl1x_distance_raw > 130)//需要测距离，离开坡道
            {
                Ramp_State = 0;
                track_state.is_ramp = false;
            }
            break;
    }
}