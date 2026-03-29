#ifndef __IPS200_H__

#include "zf_common_headfile.hpp"

#define DISP_COLOR_BLACK   0x0000
#define DISP_COLOR_WHITE   0xFFFF
#define DISP_COLOR_RED     0xF800
#define DISP_COLOR_GREEN   0x07E0
#define DISP_COLOR_BLUE    0x001F
#define DISP_COLOR_YELLOW  0xFFE0
#define DISP_COLOR_CYAN    0x07FF
#define DISP_COLOR_MAGENTA 0xF81F
#define DISP_COLOR_GRAY    0x8430

#ifdef __cplusplus
extern "C" {
#endif

// ======================= 基础控制与初始化 =======================
void screen_init(void);
void screen_clear(void);
void screen_full(uint16_t color);

// ======================= 几何图形绘制 =======================
void screen_draw_point(uint16_t x, uint16_t y, uint16_t color);
void screen_draw_line(uint16_t x_start, uint16_t y_start, uint16_t x_end, uint16_t y_end, uint16_t color);

// ======================= 字符与数据显示 =======================
void screen_show_char(uint16_t x, uint16_t y, char dat);
void screen_show_string(uint16_t x, uint16_t y, const char* dat);

// 显示整数 (dat: 数据, num: 整数部分显示的位数)
void screen_show_int(uint16_t x, uint16_t y, int32_t dat, uint8_t num);
void screen_show_uint(uint16_t x, uint16_t y, uint32_t dat, uint8_t num);

// 显示浮点数 (dat: 数据, num: 整数位数, pointnum: 小数位数)
void screen_show_float(uint16_t x, uint16_t y, double dat, uint8_t num, uint8_t pointnum);

// ======================= 波形显示 =======================
void screen_show_wave(uint16_t x, uint16_t y, const uint16_t *wave, uint16_t width, uint16_t value_max, uint16_t dis_width, uint16_t dis_value_max);

// ======================= 图像显示 (底层通用接口) =======================
void screen_show_gray_image(uint16_t x, uint16_t y, const uint8_t *image, uint16_t width, uint16_t height, uint16_t dis_width, uint16_t dis_height, uint8_t threshold);
void screen_show_rgb565_image(uint16_t x, uint16_t y, const uint16_t *image, uint16_t width, uint16_t height, uint16_t dis_width, uint16_t dis_height, uint8_t color_mode);

// ======================= 快捷图像接口 =======================
void screen_show_track(const uint8_t *img); // 固定在(0,0)显示 160x120 灰度
void screen_show_ai(const uint16_t *img);   // 满屏显示 640x480 的左上角局部

#ifdef __cplusplus
}
#endif

#endif