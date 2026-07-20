#include "usart.h"
#include <chrono>
std::chrono::time_point<std::chrono::high_resolution_clock> ua_start;
std::chrono::time_point<std::chrono::high_resolution_clock> ua_end;
static int uart_fd = -1;

static void uart_handle_special_barrier_frame(uint8_t frame_type) {
    const char *dir_name = NULL;
    switch (frame_type) {
        case 0x00:
            track_state.special_barrier_dir = DIR_LEFT;
            track_state.is_special_barrier = true;
            track_state.allow_change_special_barrier=false;
            dir_name = "LEFT";
            break;
        case 0x01:
            track_state.special_barrier_dir = DIR_NONE;
            track_state.is_special_barrier = true;
            track_state.allow_change_special_barrier=false;
            dir_name = "NONE";
            break;
        case 0x02:
            track_state.special_barrier_dir = DIR_RIGHT;
            track_state.is_special_barrier = true;
            track_state.allow_change_special_barrier=false;
            dir_name = "RIGHT";
            break;
        default:
            return;
    }

     //锁定标志
    // if(track_state.is_special_barrier){
    //     track_state.allow_change_special_barrier=false;
    // }

    printf("uart special barrier dir: %s\n", dir_name);
    fflush(stdout);
}

// 串口初始化函数
int uart_init(const char *device, int baudrate) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd == -1) {
        perror("无法打开串口设备");
        return -1;
    }

    // 恢复串口为阻塞状态，用于后续读取
    // fcntl(fd, F_SETFL, 0);

    struct termios options;
    tcgetattr(fd, &options);

    // 设置波特率
    speed_t speed;
    switch (baudrate) {
        case 9600:   speed = B9600;   break;
        case 115200: speed = B115200; break;
        case 460800: speed = B460800; break;
        default:     speed = B115200; break;
    }
    cfsetispeed(&options, speed);
    cfsetospeed(&options, speed);

    // 配置控制模式：8位数据位，无校验，1位停止位 (8N1)
    options.c_cflag &= ~PARENB;        // 无校验位
    options.c_cflag &= ~CSTOPB;        // 1位停止位
    options.c_cflag &= ~CSIZE;         // 清除当前数据位掩码
    options.c_cflag |= CS8;            // 8位数据位
    
    options.c_cflag |= (CLOCAL | CREAD); // 开启接收器

    // 配置本地模式：原始输入模式（Raw Input），不进行回显和特殊字符处理
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // 配置输入模式：禁用软件流控，不处理回车换行
    options.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL);

    // 配置输出模式：原始输出（Raw Output）
    options.c_oflag &= ~OPOST;

    // 为了让接收更实时：
    // VMIN = 1 表示 read 函数会一直阻塞，直到串口至少收到 1 个字节的数据才返回,等于0就是立刻返回
    // 这样不需要在 while 循环里用 sleep，既不浪费 CPU，又能保证只要有数据就立刻显示
    options.c_cc[VMIN] = 0;
    options.c_cc[VTIME] = 0; 

    // 清空输入输出缓冲区并使配置生效
    tcflush(fd, TCIOFLUSH);
    if (tcsetattr(fd, TCSANOW, &options) != 0) {
        perror("串口参数设置失败");
        close(fd);
        return -1;
    }

    return fd;
}

void uart_received(void) {
    //ua_start = std::chrono::high_resolution_clock::now();
    uint8_t rx_buf[256];
    static uint8_t parse_state = 0; // 0: wait AA, 1: wait BB, 2: wait type

    if (uart_fd < 0) {
        return;
    }

    memset(rx_buf, 0, sizeof(rx_buf));
        
    int rx_bytes = read(uart_fd, rx_buf, sizeof(rx_buf));
    
    // 如果不允许更改，则直接退出不读取
    // if (track_state.allow_change_special_barrier == false) {
    //     return; 
    // }

    if (rx_bytes > 0) {
        for (int i = 0; i < rx_bytes; i++) {
            uint8_t byte = rx_buf[i];

            switch (parse_state) {
                case 0:
                    if (byte == 0xAA) {
                        parse_state = 1;
                    }
                    break;
                case 1:
                    if (byte == 0xBB) {
                        parse_state = 2;
                    } else if (byte == 0xAA) {
                        parse_state = 1;
                    } else {
                        parse_state = 0;
                    }
                    break;
                case 2:
                if(track_state.allow_change_special_barrier==true){
                    uart_handle_special_barrier_frame(byte);
                    
                }   
                    //uart_send("hello\r\n");
                    parse_state = (byte == 0xAA) ? 1 : 0;
                    break;
                default:
                    parse_state = 0;
                    break;
            }
        }
    } else if (rx_bytes < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            perror("读取错误");
        }
    }
    //printf("%d\r\n",parse_state);
}

void LQ_USART_Init(void){
    const char *dev = "/dev/ttyUSB0";
    int baud = 115200; // 必须和另一块板子保持一致
    uart_fd = uart_init(dev, baud);
    if (uart_fd < 0) {
        printf("串口初始化失败\n");
    } else {
        printf("串口初始化成功\n");
    }
}

int uart_send(const char *str)
{
    if (uart_fd < 0) {
        return -1;
    }

    int len = strlen(str);
    int ret = write(uart_fd, str, len);

    if (ret < 0) {
        perror("串口发送失败");
        return -1;
    }

    return ret;
}
//uart_send("hello\r\n");

int uart_send_bytes(const uint8_t *data, int len)
{
    if (uart_fd < 0) {
        return -1;
    }

    int ret = write(uart_fd, data, len);

    if (ret < 0) {
        perror("串口发送失败");
        return -1;
    }

    return ret;
}
//uint8_t data[4] = {0xAA, 0x55, 0x01, 0xFE};
//uart_send_bytes(data, 4);

// 1. 定义为全局变量，方便初始化和读取函数都能访问
static int fd_42 = -1;
static int fd_43 = -1;
char value42, value43;

// 2. 将返回值改为 int，这样外部可以判断初始化是否成功
int LQ_GPIO_Init(void) {
    // 打开设备树中定义的节点设备（只读模式）
    fd_42 = open("/dev/zf_gpio_in_42", O_RDONLY);
    if (fd_42 < 0) {
        perror("无法打开 GPIO42 设备节点，请检查设备树");
        return -1; // 现在返回 -1 就合法了
    }
    
    fd_43 = open("/dev/zf_gpio_in_43", O_RDONLY);
    if (fd_43 < 0) {
        perror("无法打开 GPIO43 设备节点，请检查设备树");
        return -1;
    }
     printf("%d\r\n",fd_43);
    printf("GPIO 42/43 初始化成功\n");
    return 0;
}

// void GPIO_received(void) {
//     if (fd_42 < 0 || fd_43 < 0) return;

//     if (track_state.allow_change_special_barrier == false) {
//         return;
//     }
//     if (track_state.is_island == true) {
//         return;
//     }

//     lseek(fd_42, 0, SEEK_SET);
//     lseek(fd_43, 0, SEEK_SET);

//     int r42 = read(fd_42, &value42, 1);
//     int r43 = read(fd_43, &value43, 1);
//     if (r42 < 0 || r43 < 0) return; // 读取失败，跳过本次

//     bool pin42_high = (value42 == '1' || value42 == 1);
//     bool pin43_high = (value43 == '1' || value43 == 1);

//     // 1. 计算本次采样的"原始方向"
//     BypassDir_e raw_dir;
//     if (pin42_high && pin43_high) {
//         raw_dir = DIR_STRAIGHT;   // 两个都高，视为 NONE
//     } else if (pin42_high) {
//         raw_dir = DIR_LEFT;
//     } else if (pin43_high) {
//         raw_dir = DIR_RIGHT;
//     } else {
//         return; // 两个都低，不参与消抖统计
//     }

//     // 2. 计数消抖
//     static BypassDir_e candidate_dir = DIR_STRAIGHT; // 当前候选方向
//     static int stable_count = 0;                 // 连续相同次数
//     const int DEBOUNCE_THRESHOLD = 3;             // 连续5次采样一致才确认，可按采样频率调整

//     if (raw_dir == candidate_dir) {
//         if (stable_count < DEBOUNCE_THRESHOLD) {
//             stable_count++;
//         }
//     } else {
//         candidate_dir = raw_dir;
//         stable_count = 1;
//     }

//     // 3. 只有达到阈值才真正生效，且避免重复触发同一个稳定状态
//     static BypassDir_e last_confirmed_dir = DIR_STRAIGHT;
//     if (stable_count >= DEBOUNCE_THRESHOLD && candidate_dir != last_confirmed_dir) {
//         last_confirmed_dir = candidate_dir;
//         track_state.special_barrier_dir = candidate_dir;
//         track_state.is_special_barrier = true;
//         track_state.allow_change_special_barrier = false;

//         // const char *dir_name =
//         //     (candidate_dir == DIR_LEFT)  ? "LEFT"  :
//         //     (candidate_dir == DIR_RIGHT) ? "RIGHT" : "NONE";
//         //printf("PIN42=%d PIN43=%d, DIR = %s\n", pin42_high, pin43_high, dir_name);
//     }
// }


void GPIO_received(void) {
    const char *dir_name = "NONE"; // 补充缺失的变量定义
    
    // 如果设备节点没打开，直接退出防止崩溃
    if (fd_42 < 0 || fd_43 < 0) return;

    // 如果不允许更改，则直接退出不读取
    if (track_state.allow_change_special_barrier == false) {
        return; 
    }
    if (track_state.is_island==true||track_state.is_barrier==true){
        return;
    }
    // ua_start = std::chrono::high_resolution_clock::now();
    lseek(fd_42, 0, SEEK_SET);
    lseek(fd_43, 0, SEEK_SET);
    // read(fd_43, &value43, 1);
    // printf("%d\r\n",value43);
    // 读取 42 引脚 (LEFT)
    if (read(fd_42, &value42, 1) >= 0) {
        //printf("%d\r\n",value42);
        if (value42 == '1' || value42 == 1) {
            track_state.special_barrier_dir = DIR_LEFT;
            track_state.is_special_barrier = true;
            track_state.allow_change_special_barrier = false; // 触发后锁定
            dir_name = "LEFT";
            printf("PIN42 high, DIR = %s\n", dir_name);
            return; // 已经识别到左侧，直接返回，没必要再读右侧
        }
    }
    
    // 读取 43 引脚 (RIGHT)
    if (read(fd_43, &value43, 1) >= 0) {
        if (value43 == '1' || value43 == 1) {
            track_state.special_barrier_dir = DIR_RIGHT;
            track_state.is_special_barrier = true;
            track_state.allow_change_special_barrier = false; // 触发后锁定
            dir_name = "RIGHT";
            printf("PIN43 high, DIR = %s\n", dir_name);
        }
    }
}