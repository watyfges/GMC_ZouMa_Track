#include "head.h"

extern int16 imu_acc_x,imu_acc_y,imu_acc_z;
extern int16 imu_gyro_x,imu_gyro_y,imu_gyro_z;
extern float roll,pitch,yaw;
extern float V_Right, encoder_right, Right_Duty;

zf_driver_pit pit_timer; // 定义定时器设备对象
zf_driver_pit enc_pit_timer; // 定义定时器设备对象_编码器
zf_driver_pit imu_pit_timer; // 定义定时器设备对象_陀螺仪

void HardwareInit ();
void sigint_handler(int signum);

int main(int, char**) 
{
    HardwareInit ();
    control_init();
    Debug_Init(SERVER_IP, PORT_VOFA, PORT_SEEKFREE);

    //imu_pit_timer.init_ms(10, IMU_pit_callback);
    pit_timer.init_ms(10, pit_callback);
    Set_Left_Pwm(0);
    Set_Right_Pwm(0);
    while(1)
    {   
        edge_extract();
        ai_extract();
        Debug_Task();
    }
}

void HardwareInit(void){
    Motor_init();
    IMU_init();
    screen_init();
    ai_uvc_init();
    track_uvc_init();


    beep_on();
    system_delay_ms(200);
    beep_off();
}

void sigint_handler(int signum) {
    printf("收到Ctrl+C，程序即将退出\n");
    exit(0);
}