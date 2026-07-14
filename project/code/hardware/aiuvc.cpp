#include "aiuvc.h"

#include <cstring>
#include <iostream>
#include <mutex>
#include <thread>

uint16_t img_ai[AI_H][AI_W] = {0};

static zf_device_uvc ai_cam;
static std::mutex ai_mutex;
static cv::Mat ai_bgr_buffer;
static bool ai_has_frame = false;

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

static void ai_thread_func()
{
    if (ai_cam.init("/dev/video_ai", AI_S_W, AI_S_H, AI_CAMERA_FPS) != 0) {
        std::cerr << "AI Cam Init Failed!" << std::endl;
        return;
    }

    while (true) {
        if (ai_cam.wait_image_refresh() == 0) {
            cv::Mat frame = ai_cam.get_frame_mjpg();

            if (!frame.empty() && frame.cols == AI_S_W && frame.rows == AI_S_H) {
                std::lock_guard<std::mutex> lock(ai_mutex);
                ai_bgr_buffer = frame.clone();
                update_ai_rgb565_debug_buffer(ai_bgr_buffer);
                ai_has_frame = true;
            }
        }
    }
}

extern "C" int ai_uvc_init(void)
{
    std::thread t(ai_thread_func);
    t.detach();
    return 0;
}

extern "C" void ai_uvc_get_rgb_image(uint16_t *out_buffer)
{
    if (out_buffer == nullptr) {
        return;
    }

    std::lock_guard<std::mutex> lock(ai_mutex);
    std::memcpy(out_buffer, img_ai[0], AI_W * AI_H * sizeof(uint16_t));
}

bool ai_uvc_get_bgr_frame(cv::Mat &out_frame)
{
    std::lock_guard<std::mutex> lock(ai_mutex);
    if (!ai_has_frame || ai_bgr_buffer.empty()) {
        return false;
    }

    out_frame = ai_bgr_buffer.clone();
    return true;
}
