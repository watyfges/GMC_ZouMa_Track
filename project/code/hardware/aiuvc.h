#ifndef _AI_UVC_H_
#define _AI_UVC_H_

// AI识别共用巡线摄像头帧，尺寸必须与 TRACK_W/TRACK_H 保持一致
#define AI_S_W 160
#define AI_S_H 80
#define AI_W 160
#define AI_H 80

#include "zf_common_headfile.hpp"
#include <opencv2/core.hpp>

#ifdef __cplusplus
extern "C" {
#endif

// 初始化识别摄像头
int ai_uvc_init(void);

void ai_uvc_get_rgb_image(uint16_t *out_buffer);

#ifdef __cplusplus
}

bool ai_uvc_get_bgr_frame(cv::Mat &out_frame);
#endif

#endif // _AI_UVC_H_