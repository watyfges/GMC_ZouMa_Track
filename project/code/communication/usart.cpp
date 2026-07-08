#include "usart.h"

static int uart_fd = -1;

static void uart_handle_special_barrier_frame(uint8_t frame_type) {
    const char *dir_name = NULL;

    switch (frame_type) {
        case 0x00:
            track_state.special_barrier_dir = DIR_LEFT;
            track_state.is_special_barrier = true;
            dir_name = "LEFT";
            break;
        case 0x01:
            track_state.special_barrier_dir = DIR_NONE;
            track_state.is_special_barrier = true;
            dir_name = "NONE";
            break;
        case 0x02:
            track_state.special_barrier_dir = DIR_RIGHT;
            track_state.is_special_barrier = true;
            dir_name = "RIGHT";
            break;
        default:
            return;
    }

     //锁定标志
    if(track_state.is_special_barrier){
        track_state.allow_change_special_barrier=false;
    }

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
    uint8_t rx_buf[256];
    static uint8_t parse_state = 0; // 0: wait AA, 1: wait BB, 2: wait type

    if (uart_fd < 0) {
        return;
    }

    memset(rx_buf, 0, sizeof(rx_buf));
        
    int rx_bytes = read(uart_fd, rx_buf, sizeof(rx_buf));
        
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
                if(track_state.allow_change_special_barrier){
                    uart_handle_special_barrier_frame(byte);
                }
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
