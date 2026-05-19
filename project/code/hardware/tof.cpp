
#include "tof.h"

//-------------------------------------------------------------------------------------------------------------------
// 全局变量定义
//-------------------------------------------------------------------------------------------------------------------
zf_device_dl1x dl1x_dev;                  // DL1X设备对象
enum dl1x_device_type_enum dl1x_dev_type;  // DL1X设备类型
volatile int16 dl1x_distance_raw = 0;      // 定时采集的距离原始数据（volatile防止编译器优化）

//-------------------------------------------------------------------------------------------------------------------
// 函数简介  PIT定时器100ms周期回调函数
// 参数说明  无
// 返回参数  无
// 使用示例  注册到PIT定时器，定时自动执行
// 备注信息  回调函数在定时器线程中执行，需精简高效，仅做数据读取（不做耗时打印操作）
//-------------------------------------------------------------------------------------------------------------------
void tof_pit_callback(void)
{
    // 仅读取DL1X距离数据，存入全局变量（主循环负责打印，避免回调内耗时）
    if(NO_FIND_DEVICE != dl1x_dev_type)  // 仅当设备初始化成功时读取
    {
        dl1x_distance_raw = dl1x_dev.get_distance();
    }
}

void tof_init(void)             //适用于多种传感器
{
     // DL1X传感器初始化
    dl1x_dev_type = dl1x_dev.init();

    // 判断DL1X初始化结果
    if(NO_FIND_DEVICE == dl1x_dev_type)
    {
        printf("错误：未找到DL1X设备，请检查硬件连接！\r\n");
    }
    else
    {
        printf("DL1X 初始化成功！识别到：");
        if(ZF_DEVICE_DL1A == dl1x_dev_type) printf("ZF_DEVICE_DL1A\r\n");
        else if(ZF_DEVICE_DL1B == dl1x_dev_type) printf("ZF_DEVICE_DL1B\r\n");
    }
}