#include "redblock.h"

extern uint8_t L_boundary[TRACK_H], R_boundary[TRACK_H];

// ==================== 红块检测参数 ====================
// 红块在赛道中间的判定比例：红块中心x落在赛道宽度内侧 CENTER_RATIO 范围内视为"中间"
#define CENTER_RATIO 0.6f

// ==================== 红块查找核心函数 ====================
// 在 BGR 图像中寻找最优红色矩形块，返回其中心坐标 (center_x, center_y)
// 成功返回 true，center_x/center_y 为像素坐标（巡线摄像头坐标系）
static bool find_red_block(const cv::Mat &image, int &center_x, int &center_y)
{
    if (image.empty()) {
        return false;
    }

    cv::Mat hsv, low_mask, high_mask, red_mask;
    cv::cvtColor(image, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv,
                cv::Scalar(RED_H_LOW_1, RED_S_MIN, RED_V_MIN),
                cv::Scalar(RED_H_HIGH_1, RED_S_MAX, RED_V_MAX),
                low_mask);
    cv::inRange(hsv,
                cv::Scalar(RED_H_LOW_2, RED_S_MIN, RED_V_MIN),
                cv::Scalar(RED_H_HIGH_2, RED_S_MAX, RED_V_MAX),
                high_mask);
    cv::bitwise_or(low_mask, high_mask, red_mask);

    cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_RECT,
        cv::Size(RED_MORPH_KERNEL_SIZE, RED_MORPH_KERNEL_SIZE));
    cv::morphologyEx(red_mask, red_mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(red_mask, red_mask, cv::MORPH_CLOSE, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(red_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    const double frame_area = image.cols * image.rows;
    const double min_area = frame_area * RED_MIN_AREA_RATIO;
    const double max_area = frame_area * RED_MAX_AREA_RATIO;
    const int search_top = cvRound(image.rows * RED_SEARCH_TOP_RATIO);

    int best_bottom = -1;
    double best_area = 0.0;
    cv::Rect best_rect;
    bool found = false;

    for (const auto &contour : contours) {
        const double area = cv::contourArea(contour);
        if (area < min_area || area > max_area) {
            continue;
        }

        const cv::Rect rect = cv::boundingRect(contour);
        if (rect.width <= 0 || rect.height <= 0 || rect.y < search_top) {
            continue;
        }

        const int bottom = rect.y + rect.height;
        const bool lower_than_best = bottom > best_bottom + RED_BOTTOM_PRIORITY_PIXELS;
        const bool same_bottom_band = bottom + RED_BOTTOM_PRIORITY_PIXELS >= best_bottom;
        if (!found || lower_than_best || (same_bottom_band && area > best_area)) {
            best_rect = rect;
            best_bottom = bottom;
            best_area = area;
            found = true;
        }
    }

    if (found) {
        center_x = best_rect.x + best_rect.width / 2;
        center_y = best_rect.y + best_rect.height / 2;
    }
    return found;
}

// ==================== 红块障碍判定主函数 ====================
// 逻辑：
// 1. 从巡线摄像头获取 BGR 帧
// 2. 检测红块，获取中心坐标
// 3. 将红块中心 x 与该行的左右边线比较
//    - 红块在赛道中间区域 → 特殊障碍物（设置 is_barrier 等待 GPIO/串口确定绕行方向）
//    - 红块在赛道边缘   → 普通障碍物（交给 Barrier_Detect 处理）
void Red_Block_Detect()
{
    // 如果已经检测到特殊障碍且锁定中，跳过
    if (track_state.is_special_barrier && !track_state.allow_change_special_barrier) {
        return;
    }

    // 1. 获取巡线摄像头 BGR 图像
    cv::Mat bgr_frame;
    if (!track_uvc_get_bgr_frame(bgr_frame)) {
        return;
    }

    // 2. 检测红块
    int block_cx = 0, block_cy = 0;
    if (!find_red_block(bgr_frame, block_cx, block_cy)) {
        return; // 没有检测到红块，不做任何判断
    }

    // 3. 获取红块中心 y 行对应的边线（坐标系一致，直接索引）
    // 注意：cv::Mat 坐标 y 对应 img 的行号，与 boundary 数组索引一致
    if (block_cy < 0 || block_cy >= TRACK_H) {
        return;
    }

    int left  = (int)L_boundary[block_cy];
    int right = (int)R_boundary[block_cy];
    int road_width = right - left;

    if (road_width <= 0) {
        return; // 该行边线数据异常，跳过
    }

    // 4. 计算红块中心在赛道中的相对位置
    float relative_pos = (float)(block_cx - left) / (float)road_width;
    // relative_pos ∈ [0, 1]：0 = 左边线，1 = 右边线，0.5 = 赛道正中

    // 5. 判断红块是否在赛道中间
    float margin = (1.0f - CENTER_RATIO) / 2.0f; // 边缘区域宽度比例
    // 中间区域: [margin, 1 - margin]

    if (relative_pos > margin && relative_pos < (1.0f - margin)) {
        // 红块在赛道中间 → 特殊障碍物
        // 标记为 barrier 状态，等待 GPIO/串口告知绕行方向
        track_state.is_barrier = true;

        printf("[RedBlock] SPECIAL barrier detected! cx=%d, cy=%d, rel=%.2f\n",
               block_cx, block_cy, relative_pos);
    }
    // 红块在赛道边缘 → 普通障碍物，不在这里处理，由 Barrier_Detect() 通过边线判断
}