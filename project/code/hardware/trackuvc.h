#ifndef _TRACK_UVC_H_
#define _TRACK_UVC_H_

#define TRACK_W 160
#define TRACK_H 80
#define TRACK_CAMERA_FPS 60

#include "zf_common_headfile.hpp"
#include <opencv2/core.hpp>

#ifdef __cplusplus
extern "C" {
#endif

// 初始化巡线摄像头 (返回 0 表示成功)
int track_uvc_init(void);

// 获取最新的一帧巡线灰度图
// 参数 out_buffer: 由C语言传入的数组指针，大小必须是 160 * 80 字节
void track_uvc_get_image(uint8_t *out_buffer);

#ifdef __cplusplus
}

bool track_uvc_get_bgr_frame(cv::Mat &out_frame);
#endif

#endif // _TRACK_UVC_H_
