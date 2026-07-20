#include "zebra.h"
extern bool Departure;   //发车标志
extern uint8 img_bin[TRACK_H][TRACK_W];//二值化的图像

void Zebra_Check(void){
    track_state.is_zebra = false;
    for(int i = LOWEST_ROW - TRACK_H / 2 ;i < LOWEST_ROW; i++){
        int Stop_num = 0; //停止线标志位
        for(int j = 0; j < TRACK_W - 1; j++)
        {
            if(img_bin[i][j] != img_bin[i][j+1]){
                Stop_num ++; //如果当前像素与下一个像素不同，则停止线标志位置为1
            }
        }
        if(Stop_num >= 14)
        {
            if(!track_state.is_island && !track_state.is_cross && Departure==true)
            {
                //printf("banma?\r\n");
                track_state.stop_flag = 0; //如果检测到停止线，则停车
            }
                track_state.is_zebra = true; //如果检测到斑马线，则斑马线标志位置为1
            break;
        }
    }
}