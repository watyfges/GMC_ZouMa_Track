#include "barrier.h"

extern uint8 L_boundary[TRACK_H], M_boundary[TRACK_H], R_boundary[TRACK_H]; //左右边界

extern int Road_width[TRACK_H];

char Barrier_Dir = 'N';//L:左侧障碍物 R:右侧障碍物 N:无障碍物

bool Barrier_check()//检测障碍物
{
        //逻辑：检测道路中线连续性，如果存在断点使得中线上下偏差过大，再判断左右边线的连续性，判断障碍物在左侧还是右侧，并返回标志位
        for(int i = LOWEST_ROW; i >= LOWEST_ROW - 40; i--)
        {
            M_boundary[i] = (R_boundary[i] + L_boundary[i]) / 2;
        }  

        for(int i = LOWEST_ROW; i >= LOWEST_ROW - 40; i--)
        {
            int error = fabs((int)M_boundary[i] - (int)M_boundary[i - 2]);
            if(error > 5 && error < 40){
                if(//(Road_width[i] - ((int)R_boundary[i] - (int)L_boundary[i])) >= 10 && 
                //(Road_width[i - 1] - ((int)R_boundary[i - 1] - (int)L_boundary[i - 1])) >= 10 && 
                //(Road_width[i - 3] - ((int)R_boundary[i - 3] - (int)L_boundary[i - 3])) >= 10 &&
                R_boundary[i] != TRACK_W - 1 && L_boundary[i] != 0 &&
                R_boundary[i - 2] != TRACK_W - 1 && L_boundary[i - 2] != 0 &&
                R_boundary[i - 4] != TRACK_W - 1 && L_boundary[i - 4] != 0 &&
                R_boundary[i + 2] != TRACK_W - 1 && L_boundary[i + 2] != 0 &&
                R_boundary[i + 4] != TRACK_W - 1 && L_boundary[i + 4] != 0 ){
                    //printf("!\r\n");
                    //printf("[%d:%d]\r\n",(int)R_boundary[i] , (int)R_boundary[i - 2]);
                    if((int)L_boundary[i] - (int)L_boundary[i - 2] <= -10 && 
                    abs((int)L_boundary[i] - (int)L_boundary[i+1]) <= 5 &&
                    abs((int)L_boundary[i+1] - (int)L_boundary[i+2]) <= 5 &&
                    abs((int)L_boundary[i+2] - (int)L_boundary[i+3]) <= 5 && 
                    abs((int)L_boundary[i-2] - (int)L_boundary[i-3]) <= 5 &&
                    abs((int)L_boundary[i-3] - (int)L_boundary[i-4]) <= 5 &&
                    abs((int)L_boundary[i-4] - (int)L_boundary[i-5]) <= 5 ){
                        Barrier_Dir = 'L';
                        return 1;
                    }
                    else if((int)R_boundary[i] - (int)R_boundary[i - 2] >= 10 &&
                    abs((int)R_boundary[i] - (int)R_boundary[i+1]) <= 5 &&
                    abs((int)R_boundary[i+1] - (int)R_boundary[i+2]) <= 5 &&
                    abs((int)R_boundary[i+2] - (int)R_boundary[i+3]) <= 5 && 
                    abs((int)R_boundary[i-2] - (int)R_boundary[i-3]) <= 5 &&
                    abs((int)R_boundary[i-3] - (int)R_boundary[i-4]) <= 5 &&
                    abs((int)R_boundary[i-4] - (int)R_boundary[i-5]) <= 5 ){
                        Barrier_Dir = 'R';
                        return 1;
                    }
                } 
            }
        }
    return 0;
}

bool EX_Barrier(char Dir)//检测障碍物是否通过
{
    //逻辑：检测障碍物侧边线的连续性，如果侧边线恢复正常则认为障碍物通过
    if(Dir == 'R')
    {
        for(int i = LOWEST_ROW; i >= LOWEST_ROW - 25; i--)
        {
            if(R_boundary[i] < R_boundary[i - 2] || 
            abs((int)R_boundary[i] - (int)R_boundary[i - 1]) > 5 ||
            abs((int)L_boundary[i] - (int)L_boundary[i - 1]) > 5 ){
                return 0;
            }
        }
    }
    else if(Dir == 'L')
    {
        for(int i = LOWEST_ROW; i >= LOWEST_ROW - 25; i--)
        {
            if(L_boundary[i] > L_boundary[i - 2] || 
            abs((int)R_boundary[i] - (int)R_boundary[i - 1]) > 5 ||
            abs((int)L_boundary[i] - (int)L_boundary[i - 1]) > 5 ){
                return 0;
            }
        }
    }
    return 1;
}

void Barrier_Detect(void)//检测障碍物
{
    // 特殊障碍物生效时不做普通障碍检测，避免冲突
    if(track_state.is_special_barrier) return;

    if(!track_state.is_barrier){//无障碍物状态
        if(Barrier_check()){
            track_state.is_barrier = true;
        }
    }
    else{
        if(EX_Barrier(Barrier_Dir)){//障碍物通过
            track_state.is_barrier = false;
            Barrier_Dir = 'N';
        }
        if(Barrier_Dir == 'R')//右侧障碍物
        {
            for(int i = 0; i < TRACK_H; ++i){
                R_boundary[i] = L_boundary[i] + Road_width[i] / 2;
            }
        }
        else if(Barrier_Dir == 'L')//左侧障碍物
        {
            for(int i = 0; i < TRACK_H; ++i){
                L_boundary[i] = R_boundary[i] - Road_width[i] / 2;
            }
        }

    }
}