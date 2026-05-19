#include "imu.h"
//本程序包含数据处理，姿态计算部分，由标志位控制
//通过加速度计和陀螺仪数据融合计算欧拉角（横滚roll、俯仰pitch、偏航yaw）
//想陀螺仪准确需要调整pid参数

/**
 * @brief 通过加速度计和陀螺仪数据融合计算欧拉角（横滚roll、俯仰pitch、偏航yaw）
 * @param roll3  输出横滚角（单位：度）
 * @param pitch3 输出俯仰角（单位：度）
 * @param yaw3   输出偏航角（单位：度）
 */
#define Kp 0.2f // 加速度权重
#define Ki 0.001f // 误差积分增益
#define DT 0.005f // 采样时间间隔

zf_device_imu imu_dev;//定义IMU传感器设备对象

// 定义传感器数据结构体
typedef struct {
float q0, q1, q2, q3;
} Quat;

typedef struct {
float gx, gy, gz; // 陀螺仪测量值
float ax, ay, az; // 加速度计测量值
} SensorData;

//全局变量-IMU六轴/九轴传感器原始数据存储变量
int16 imu_acc_x,imu_acc_y,imu_acc_z;
int16 imu_gyro_x,imu_gyro_y,imu_gyro_z;
int16 imu_mag_x,imu_mag_y,imu_mag_z;

Quat chj = {1.0f,0,0,0};//初始化
SensorData uef = {0};//初始化


//初始化各变量*（数据处理用的）
static float ax = 0, ay = 0, az = 0;
static float bx = 0, by = 0, bz = 0;
static float ax_prev = 0, ay_prev = 0, az_prev = 0;
static float bx_prev = 0, by_prev = 0, bz_prev = 0;
static float exInt_prev = 0, eyInt_prev = 0, ezInt_prev = 0;
float exInt, eyInt, ezInt;
float gx_corr=0,gy_corr=0,gz_corr=0;
float q0_dot =0 , q1_dot=0,q2_dot=0,q3_dot=0;
float norm, vx, vy, vz, ex, ey, ez;
float pp,roll,pitch,yaw;   //处理后的的数据变量
float GZ;


void IMU_pit_callback()
{
    if(DEV_IMU660RA == imu_dev.imu_type || DEV_IMU660RB == imu_dev.imu_type)
    {
        imu_acc_x = imu_dev.get_acc_x();
        imu_acc_y = imu_dev.get_acc_y();
        imu_acc_z = imu_dev.get_acc_z();

        imu_gyro_x = imu_dev.get_gyro_x();
        imu_gyro_y = imu_dev.get_gyro_y();
        imu_gyro_z = imu_dev.get_gyro_z();
    }
    // 读取九轴传感器(IMU963RA)数据:加速度+陀螺仪+磁力计
    else if(DEV_IMU963RA == imu_dev.imu_type)
    {
        imu_acc_x = imu_dev.get_acc_x();
        imu_acc_y = imu_dev.get_acc_y();
        imu_acc_z = imu_dev.get_acc_z();

        imu_gyro_x = imu_dev.get_gyro_x();
        imu_gyro_y = imu_dev.get_gyro_y();
        imu_gyro_z = imu_dev.get_gyro_z();

        imu_mag_x = imu_dev.get_mag_x();
        imu_mag_y = imu_dev.get_mag_y();
        imu_mag_z = imu_dev.get_mag_z();
    }
    GZ = imu_gyro_z;
    Angle_seem();
}

void IMU_init(void)             //适用于多种传感器
{
    imu_dev.init();
    
    if(DEV_IMU660RA == imu_dev.imu_type)
    {
        printf("IMU DEV IS IMU660RA\r\n");printf("OK\r\n");
    }
    else if(DEV_IMU660RB == imu_dev.imu_type)
    {
        printf("IMU DEV IS IMU660RB\r\n");
    }
    else if(DEV_IMU963RA == imu_dev.imu_type)
    {
        printf("IMU DEV IS IMU963RA\r\n");
    }
    else
    {
        printf("NO FIND IMU DEV\r\n");
        return;
    }
    //pit_timer.init_ms(5, IMU_pit_callback);  //注册5ms定时器
}


void Angle_seem()
{
    // 2. 传感器数据滤波处理
    // 加速度计一阶低通滤波（系数0.8为新数据权重，0.2为历史数据权重）
    ax = imu_acc_x * 0.8 + 0.2 * ax_prev;
    ay = imu_acc_y * 0.8 + 0.2 * ay_prev;
    az = imu_acc_z * 0.8 + 0.2 * az_prev;
    ax_prev = ax; // 更新加速度历史值
    ay_prev = ay;
    az_prev = az;
 
    // 陀螺仪数据处理：
    // - 0.07f：陀螺仪量程转换系数（假设传感器量程为±2000dps时的LSB/dps值）
    // - 0.0174532925f：度转弧度系数（π/180）
    // - 一阶低通滤波（系数同上）
    bx = (0.07f * imu_gyro_x) * 0.0174532925f * 0.8 + 0.2 * bx_prev;
    by = (0.07f * imu_gyro_y) * 0.0174532925f * 0.8 + 0.2 * by_prev;
    bz = (0.07f * imu_gyro_z) * 0.0174532925f * 0.8 + 0.2 * bz_prev;
    bx_prev = bx; // 更新角速度历史值
    by_prev = by;
    bz_prev = bz;
 
    // 3. 加速度归一化处理（将加速度向量转换为单位向量）
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm < 1e-6) {
        return; // 加速度异常，直接退出本次解算，保持上一周期的姿态
    }
    ax /= norm;
    ay /= norm;
    az /= norm;
 
    // 异常处理：加速度过小时跳过计算
    if (norm < 1e-6) {
        ax = 0; ay = 0; az = 0;
        return;
    }
 
    // 4. 计算预测的重力方向（通过当前四元数）
    // 重力向量在机体坐标系中的投影
    vx = 2 * (chj.q1 * chj.q3 - chj.q0 * chj.q2);  // 2(q1q3 - q0q2)
    vy = 2 * (chj.q0 * chj.q1 + chj.q2 * chj.q3);  // 2(q0q1 + q2q3)
    vz = chj.q0 * chj.q0 - chj.q1 * chj.q1 - chj.q2 * chj.q2 + chj.q3 * chj.q3; // q0² - q1² - q2² + q3²
 
    // 5. 计算加速度计测量值与预测值的误差（叉积）
    ex = ay * vz - az * vy;  // 横滚轴误差
    ey = az * vx - ax * vz;  // 俯仰轴误差
    ez = ax * vy - ay * vx;  // 偏航轴误差
 
    // 6. 误差积分（用于消除稳态误差）
    exInt = exInt_prev + ex * Ki * DT;  // Ki为积分系数，DT为采样周期
    eyInt = eyInt_prev + ey * Ki * DT;
    ezInt = ezInt_prev + ez * Ki * DT;
    exInt_prev = exInt;  // 保存积分值
    eyInt_prev = eyInt;
    ezInt_prev = ezInt;
 
    // 7. 陀螺仪数据修正（比例-积分反馈）
    gx_corr = bx + Kp * ex + exInt;  // Kp为比例系数
    gy_corr = by + Kp * ey + eyInt;
    gz_corr = bz + Kp * ez + ezInt;
 
    // 8. 四元数更新（微分方程离散化）
    // 四元数导数计算
    q0_dot = -chj.q1 * gx_corr - chj.q2 * gy_corr - chj.q3 * gz_corr;
    q1_dot =  chj.q0 * gx_corr + chj.q2 * gz_corr - chj.q3 * gy_corr;
    q2_dot =  chj.q0 * gy_corr - chj.q1 * gz_corr + chj.q3 * gx_corr;
    q3_dot =  chj.q0 * gz_corr + chj.q1 * gy_corr - chj.q2 * gx_corr;
 
    // 四元数积分更新（0.5*DT为离散化系数）
    chj.q0 += q0_dot * DT * 0.5f;
    chj.q1 += q1_dot * DT * 0.5f;
    chj.q2 += q2_dot * DT * 0.5f;
    chj.q3 += q3_dot * DT * 0.5f;
 
    // 9. 四元数归一化（避免数值误差累积）
    pp = sqrt(chj.q0 * chj.q0 + chj.q1 * chj.q1 + chj.q2 * chj.q2 + chj.q3 * chj.q3);
    chj.q0 /= pp;
    chj.q1 /= pp;
    chj.q2 /= pp;
    chj.q3 /= pp;
 
    // 10. 四元数转欧拉角（Z-Y-X旋转顺序）
    // 横滚角（绕X轴旋转）
    roll = atan2(2 * (chj.q2 * chj.q3 + chj.q0 * chj.q1), 
                chj.q0 * chj.q0 - chj.q1 * chj.q1 - chj.q2 * chj.q2 + chj.q3 * chj.q3) * 180 / 3.1415926;
    // 俯仰角（绕Y轴旋转）
    pitch = asin(-2 * (chj.q1 * chj.q3 - chj.q0 * chj.q2)) * 180 / 3.1415926;
    // 偏航角（绕Z轴旋转）
    yaw = atan2(2 * (chj.q1 * chj.q2 + chj.q0 * chj.q3), 
               chj.q0 * chj.q0 + chj.q1 * chj.q1 - chj.q2 * chj.q2 - chj.q3 * chj.q3) * 180 / 3.1415926;
}
