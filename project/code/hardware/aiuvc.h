#ifndef _AI_UVC_H_
#define _AI_UVC_H_

#define AI_W 480
#define AI_H 480

#include "zf_common_headfile.hpp"

#ifdef __cplusplus
extern "C" {
#endif

// 初始化识别摄像头
int ai_uvc_init(void);

void ai_uvc_get_rgb_image(uint16_t *out_buffer);

#ifdef __cplusplus
}
#endif

#endif // _AI_UVC_H_