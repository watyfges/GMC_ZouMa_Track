#include "head.h"

extern int16 imu_acc_x,imu_acc_y,imu_acc_z;
extern int16 imu_gyro_x,imu_gyro_y,imu_gyro_z;
extern float roll,pitch,yaw;
extern float V_Right, encoder_right, Right_Duty;
extern float V_Left,encoder_left;
extern volatile int16 dl1x_distance_raw;

zf_driver_pit pit_timer; // 定义定时器设备对象
zf_driver_pit imu_pit_timer; // 定义定时器设备对象_陀螺仪
zf_driver_pit tcp_pit_timer; // 定义定时器设备对象_TCP
zf_driver_pit tof_pit_timer; // 定义定时器设备对象_tof
zf_driver_pit brushless_pit_timer; // 定义定时器设备对象_无刷电机

//zf_driver_pit enc_pit_timer;

void HardwareInit ();
void sigint_handler(int signum);

extern uint8_t img_bin[TRACK_H][TRACK_W];

int main(int, char**) 
{   
    HardwareInit ();
    Debug_Init(SERVER_IP, PORT_VOFA, PORT_SEEKFREE);
    //ai_init();

    //定时器注册
    imu_pit_timer.init_ms(10, IMU_pit_callback);
    pit_timer.init_ms(2, pit_callback);
    tcp_pit_timer.init_ms(1,tcp_pit_callback);
    tof_pit_timer.init_ms(20, tof_pit_callback);
    brushless_pit_timer.init_ms(10, brushless_pit_callback);
    //enc_pit_timer.init_ms(2,encoder_pit_callback);
    

    track_state.stop_flag = 0;
    while(1)
    {   
        if(track_state.stop_flag == 0)//停车
        {
            edge_extract(); 
            //ai_extract();
            control_init();

            // track_state.is_special_barrier=true;
            // track_state.special_barrier_dir=DIR_LEFT;

            Debug_Task();
            uart_received();
            ips_draw();

            //Vofa_Send_Waveform(roll,pitch,yaw,0);
            //printf("ax:%d\r\n, ay:%d\r\n, az:%d\r\n, gx:%d\r\n, gy:%d\r\n, gz:%d\r\n, roll:%f\r\n, pitch:%f\r\n, yaw:%f\r\n",imu_acc_x,imu_acc_y,imu_acc_z,imu_gyro_x,imu_gyro_y,imu_gyro_z,roll,pitch,yaw);
        }
        else if(track_state.stop_flag == 1)
        {
            edge_extract();
            //ai_extract();
            uart_received();
            control_init();
            // track_state.is_special_barrier=true;
            // track_state.special_barrier_dir=DIR_LEFT;

            //Vofa_Send_Waveform(roll,pitch,yaw,0);
            //Vofa_Send_Waveform(V_Right, encoder_right, V_Left,encoder_left);
            Debug_Task();
        }
        else if(track_state.stop_flag == 2)//2启动，1为预备发车
        {
            
            edge_extract();
            //ai_extract();
            uart_received();

            // track_state.is_special_barrier=true;
            // track_state.special_barrier_dir=DIR_LEFT;

            //Vofa_Send_Waveform(roll,pitch,yaw,0);
            //Vofa_Send_Waveform(V_Right, encoder_right, V_Left,encoder_left);
            Debug_Task();            

        }
    }
}

void HardwareInit(void){
    Motor_init();
    IMU_init();
    tof_init();
    screen_init();
    //ai_uvc_init();
    track_uvc_init();
    brushless_init();
    LQ_USART_Init();


    beep_on();
    system_delay_ms(200);
    beep_off();
}

void sigint_handler(int signum) {
    printf("收到Ctrl+C，程序即将退出\n");
    exit(0);
}