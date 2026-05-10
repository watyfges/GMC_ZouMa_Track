#include "aiuvc.h"

static zf_device_uvc ai_cam;
static std::mutex ai_mutex;
static uint16_t ai_rgb_buffer[AI_W * AI_H]; 

static void ai_thread_func() 
{
    if (ai_cam.init("/dev/video_ai", AI_W, AI_H, 120) != 0) return;

    while (true) {
        if (ai_cam.wait_image_refresh() == 0) {
            // 先获取原始图像帧，检查它的尺寸是否完整！
            cv::Mat frame = ai_cam.get_frame_mjpg();
            
            // 只有当长宽完完全全等于 480x480 时，才允许进行内存拷贝
            if (!frame.empty() && frame.cols == AI_W && frame.rows == AI_H) 
            {
                uint16_t* raw_rgb = ai_cam.get_rgb_image_ptr();
                if (raw_rgb != nullptr) {
                    std::lock_guard<std::mutex> lock(ai_mutex);
                    std::memcpy(ai_rgb_buffer, raw_rgb, AI_W * AI_H * sizeof(uint16_t));
                }
            }
            else 
            {
                // 丢弃因为 USB 带宽不足导致损坏的残缺帧，防止 memcpy 越界崩溃
                // printf("[AI Camera] 丢弃损坏帧: %dx%d\n", frame.cols, frame.rows);
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
    if (out_buffer == nullptr) return;
    std::lock_guard<std::mutex> lock(ai_mutex);
    std::memcpy(out_buffer, ai_rgb_buffer, AI_W * AI_H * sizeof(uint16_t));
}