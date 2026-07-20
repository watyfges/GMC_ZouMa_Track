#include "trackuvc.h"

static zf_device_uvc track_cam;
static std::mutex track_mutex;
static uint8_t track_gray_buffer[TRACK_W * TRACK_H]; // 内部安全缓存
static cv::Mat track_bgr_buffer;
static bool track_has_bgr_frame = false;

// 独立的抓取线程函数
static void track_thread_func() 
{
    // 初始化 video2
    if (track_cam.init("/dev/video_track", TRACK_W, TRACK_H, 180) != 0) {
        std::cerr << "Track Cam Init Failed!" << std::endl;
        return;
    }

    while (true) {
        if (track_cam.wait_image_refresh() == 0) {
            // 获取灰度图指针（库内部已经做好了 BGR2GRAY 转换）
            uint8_t* raw_gray = track_cam.get_gray_image_ptr();
            
            if (raw_gray != nullptr) {
                // 加锁，将底层更新的图像拷贝到安全缓存区
                std::lock_guard<std::mutex> lock(track_mutex);
                std::memcpy(track_gray_buffer, raw_gray, TRACK_W * TRACK_H);
                track_bgr_buffer = raw_bgr.clone();
                track_has_bgr_frame = true;
            }
        }
    }
}

// 供 C 语言调用的初始化函数
extern "C" int track_uvc_init(void) 
{
    // 启动线程并分离（让它在后台死循环跑）
    std::thread t(track_thread_func);
    t.detach();
    return 0;
}

// 供 C 语言调用的获取图像函数
extern "C" void track_uvc_get_image(uint8_t *out_buffer) 
{
    if (out_buffer == nullptr) return;
    
    // 加锁，将缓存区的图像拷贝给 C 语言的算法数组
    std::lock_guard<std::mutex> lock(track_mutex);
    std::memcpy(out_buffer, track_gray_buffer, TRACK_W * TRACK_H);
}

bool track_uvc_get_bgr_frame(cv::Mat &out_frame)
{
    std::lock_guard<std::mutex> lock(track_mutex);
    if (!track_has_bgr_frame || track_bgr_buffer.empty()) {
        return false;
    }

    out_frame = track_bgr_buffer.clone();
    return true;
}