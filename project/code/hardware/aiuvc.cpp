#include "aiuvc.h"
#include "trackuvc.h"

#include <cstring>

uint16_t img_ai[AI_H][AI_W] = {0};

static void update_ai_rgb565_debug_buffer(const cv::Mat &frame)
{
    cv::Mat rgb565;
    cv::cvtColor(frame, rgb565, cv::COLOR_BGR2BGR565);

    if (rgb565.isContinuous()) {
        std::memcpy(img_ai[0], rgb565.data, AI_W * AI_H * sizeof(uint16_t));
        return;
    }

    for (int y = 0; y < AI_H; ++y) {
        std::memcpy(img_ai[y], rgb565.ptr(y), AI_W * sizeof(uint16_t));
    }
}

extern "C" int ai_uvc_init(void)
{
    return 0;
}

extern "C" void ai_uvc_get_rgb_image(uint16_t *out_buffer)
{
    if (out_buffer == nullptr) {
        return;
    }

    std::memcpy(out_buffer, img_ai[0], AI_W * AI_H * sizeof(uint16_t));
}

bool ai_uvc_get_bgr_frame(cv::Mat &out_frame)
{
    if (!track_uvc_get_bgr_frame(out_frame)) {
        return false;
    }

    update_ai_rgb565_debug_buffer(out_frame);
    return true;
}
