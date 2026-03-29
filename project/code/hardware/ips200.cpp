#include "ips200.h"

static zf_device_ips200 ips200;

extern "C" {

// -------------------------------------------------------------------------
// 基础控制与初始化
// -------------------------------------------------------------------------

// 屏幕初始化
void screen_init(void) {
    // 调用底层初始化，固定挂载设备节点 /dev/fb0，参数1表示重新加载底层驱动
    ips200.init("/dev/fb0", 1);
}

// 清除全屏 (默认清为空白背景色)
void screen_clear(void) {
    ips200.clear();
}

// 填充全屏为指定颜色
void screen_full(uint16_t color) {
    ips200.full(color);
}


// -------------------------------------------------------------------------
// 几何图形绘制
// -------------------------------------------------------------------------

// 在指定坐标 (x, y) 绘制一个颜色为 color 的像素点
void screen_draw_point(uint16_t x, uint16_t y, uint16_t color) {
    ips200.draw_point(x, y, color);
}

// 绘制一条直线 (起点到终点)
void screen_draw_line(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, uint16_t color) {
    ips200.draw_line(x_start, y_start, x_end, y_end, color);
}


// -------------------------------------------------------------------------
// 字符与数据显示
// -------------------------------------------------------------------------

// 显示单个 ASCII 字符 (默认使用库内置的 8x16 字体)
void screen_show_char(uint16_t x, uint16_t y, char dat) {
    ips200.show_char(x, y, dat);
}

// 显示字符串
void screen_show_string(uint16_t x, uint16_t y, const char* dat) {
    ips200.show_string(x, y, dat);
}

// 显示有符号整数 (支持负数)
// num: 限定整数显示的位数，不足补空格，超出截断
void screen_show_int(uint16_t x, uint16_t y, int32_t dat, uint8_t num) {
    ips200.show_int(x, y, dat, num);
}

// 显示无符号整数 (只有正数)
void screen_show_uint(uint16_t x, uint16_t y, uint32_t dat, uint8_t num) {
    ips200.show_uint(x, y, dat, num);
}

// 显示浮点数
// num: 整数部分显示的位数
// pointnum: 小数部分显示的位数
void screen_show_float(uint16_t x, uint16_t y, double dat, uint8_t num, uint8_t pointnum) {
    ips200.show_float(x, y, dat, num, pointnum);
}


// -------------------------------------------------------------------------
// 波形与通用图像显示
// -------------------------------------------------------------------------

// 显示一维波形图 (常用于调试 PID 曲线等)
void screen_show_wave(uint16_t x, uint16_t y, const uint16_t *wave, uint16_t width, uint16_t value_max, uint16_t dis_width, uint16_t dis_value_max) {
    ips200.show_wave(x, y, wave, width, value_max, dis_width, dis_value_max);
}

// 通用灰度图像显示接口
// threshold: 二值化阈值，填 0 则直接显示正常灰度画面
void screen_show_gray_image(uint16_t x, uint16_t y, const uint8_t *image, uint16_t width, uint16_t height, uint16_t dis_width, uint16_t dis_height, uint8_t threshold) {
    if(image != nullptr) {
        ips200.show_gray_image(x, y, image, width, height, dis_width, dis_height, threshold);
    }
}

// 通用彩色图像显示接口
// color_mode: 0为低位在前，1为高位在前 (取决于摄像头硬件，显示颜色异常时可切换此值)
void screen_show_rgb565_image(uint16_t x, uint16_t y, const uint16_t *image, uint16_t width, uint16_t height, uint16_t dis_width, uint16_t dis_height, uint8_t color_mode) {
    if(image != nullptr) {
        ips200.show_rgb565_image(x, y, image, width, height, dis_width, dis_height, color_mode);
    }
}


// -------------------------------------------------------------------------
// 项目快捷图像接口 (基于当前摄像头参数高度定制)
// -------------------------------------------------------------------------

// 快捷接口：显示巡线摄像头画面
// 固定将 160x120 的灰度图显示在屏幕原点 (0,0) 位置
void screen_show_track(const uint8_t *img) {
    if(img != nullptr) {
        // 调用底层，原始宽高 160x120，显示范围也是 160x120
        ips200.show_gray_image(0, 0, img, 160, 120, 160, 120, 0);
    }
}

// 快捷接口：显示 AI 识别摄像头画面
// 由于屏幕只有 320x240，这里固定将 640x480 的画面塞进显示接口，
// 底层 draw_point 函数的安全边界锁会自动丢弃越界像素，起到"自动裁剪"的效果。
void screen_show_ai(const uint16_t *img) {
    if(img != nullptr) {
        ips200.show_rgb565_image(0, 0, img, 640, 480, 640, 480, 0);
    }
}

} // end extern "C"