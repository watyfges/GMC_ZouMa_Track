#include "button.h"

// 1. 宏定义
#define KEY_1_PATH        ZF_GPIO_KEY_1
#define KEY_2_PATH        ZF_GPIO_KEY_2
#define KEY_3_PATH        ZF_GPIO_KEY_3
#define KEY_4_PATH        ZF_GPIO_KEY_4

// 2. 实例化驱动对象
static zf_driver_gpio  key_1(KEY_1_PATH, O_RDWR);
static zf_driver_gpio  key_2(KEY_2_PATH, O_RDWR);
static zf_driver_gpio  key_3(KEY_3_PATH, O_RDWR);
static zf_driver_gpio  key_4(KEY_4_PATH, O_RDWR);

// 3. 统一读取函数
int scan_key(int num)
{
    switch(num)
    {
        case 1:
            return key_1.get_level();
        case 2:
            return key_2.get_level();
        case 3:
            return key_3.get_level();
        case 4:
            return key_4.get_level();
        default:
            break;
    }
    
    return -1;
}