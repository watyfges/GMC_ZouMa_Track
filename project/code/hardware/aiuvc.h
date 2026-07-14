#ifndef _AI_UVC_H_
#define _AI_UVC_H_

// AI 摄像头配置，与当前 NCNN3 识别程序保持一致
#define AI_S_W 320
#define AI_S_H 240
#define AI_W 320
#define AI_H 240
#define AI_CAMERA_FPS 120

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