#include "aiuvc.h"

static zf_device_uvc ai_cam;
static std::mutex ai_mutex;
// 缓存大小保持不变，因为 640*480 和 480*640 占用的总像素数是一样的
static uint16_t ai_rgb_buffer[AI_S_W * AI_S_H]; 

static void ai_thread_func() 
{
    if (ai_cam.init("/dev/video_ai", AI_S_W, AI_S_H, 400) != 0) {
        std::cerr << "AI Cam Init Failed!" << std::endl;
        return;
    }

    while (true) {
        if (ai_cam.wait_image_refresh() == 0) {
            // 获取原始图像帧，检查它的尺寸
            cv::Mat frame = ai_cam.get_frame_mjpg();
            
            // 只有当长宽完完全全等于 (AI_S_W x AI_S_H) 时，才允许处理
            if (!frame.empty() && frame.cols == AI_S_W && frame.rows == AI_S_H) 
            {
                uint16_t* raw_rgb = ai_cam.get_rgb_image_ptr();
                if (raw_rgb != nullptr) {
                    
                    // 1. 将 16-bit raw 数组包装为 OpenCV Mat
                    // 妙招：用 CV_16UC1 (16位无符号单通道) 欺骗 OpenCV，
                    // 让它只管搬运 16-bit 像素块，不管具体颜色格式 (如 RGB565)
                    cv::Mat src(AI_S_H, AI_S_W, CV_16UC1, raw_rgb);
                    cv::Mat rotated;

                    // 2. 顺时针旋转 90 度
                    // 此时 rotated 的尺寸将变成 AI_S_W (高640) x AI_S_H (宽480)
                    cv::rotate(src, rotated, cv::ROTATE_90_CLOCKWISE);

                    // 3. 上锁并拷贝旋转后的数据
                    std::lock_guard<std::mutex> lock(ai_mutex);
                    // 总字节数没变，直接从 rotated.data 拷贝到静态数组
                    std::memcpy(ai_rgb_buffer, rotated.data, AI_S_W * AI_S_H * sizeof(uint16_t));
                }
            }
            else 
            {
                // 丢弃因为 USB 带宽不足导致损坏的残缺帧，防止 memcpy 越界崩溃
                //printf("[AI Camera] 丢弃损坏帧: %dx%d\n", frame.cols, frame.rows);
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
    // 假设你在外部定义的 AI_W 是 480，AI_H 是 640
    // 这里拷贝的总大小与 ai_rgb_buffer 吻合
    std::memcpy(out_buffer, ai_rgb_buffer, AI_W * AI_H * sizeof(uint16_t));
}