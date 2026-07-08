#include "menu.h"
extern uint8_t img_bin[TRACK_H][TRACK_W];  // 巡线二值化图像
extern uint8_t Group;
extern float Speed;
extern int Max_01_value;
void ips_draw(){
    screen_show_gray_image(0, 0, img_bin[0], TRACK_W, TRACK_H, TRACK_W, TRACK_H, 0);

    screen_show_string(0,100,"Group:");
    screen_show_int(80,100,Group,1);

    screen_show_string(0,130,"Speed:");
    screen_show_float(80,130,Speed,3,1);

    screen_show_string(0,160,"Value:");
    screen_show_int(80,160,Max_01_value,3);

    
    //ips200_show_gray_image(0, 180, img_bin[0], TRACK_W, TRACK_H);//显示图像
} 