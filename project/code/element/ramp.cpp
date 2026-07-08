#include "ramp.h"

/*
 * @brief 赛道连续性检测 (辅助函数)
 * @return true: 连续; false: 断裂(可能是十字/断路)
 */
static bool check_line_continuity(int start_y, int end_y) 
{
    for (int i = start_y; i >= end_y; i--) {
        // 如果相邻两行横坐标跳变超过 5 个像素，认为断线
        if (abs(L_boundary[i] - L_boundary[i+1]) > 5 || 
            abs(R_boundary[i] - R_boundary[i+1]) > 5) {
            return false;
        }
    }
    return true;
}

/*
 * @brief 流水线调用的坡道视觉检测
 * @note  利用坡道底部的透视畸变（赛道收缩率突变）来判定
 * @return true: 前方有坡道; false: 无坡道
 */
bool Ramp_Detect(void)
{
    // 1. 基础条件筛查
    // 确保近端有足够宽的赛道 (排除噪点或极窄弯道)
    int near_y = LOWEST_ROW - 10;
    if (R_boundary[near_y] - L_boundary[near_y] < 60) {
        return false;
    }

    // 确保视野前方有一定的通透度
    if (track_state.longest_white_len < 50) {
        return false;
    }

    // 检查赛道边线是否连续 (坡道边线是不会断的)
    if (!check_line_continuity(LOWEST_ROW - 10, LOWEST_ROW - 40)) {
        return false;
    }

    // 2. 透视畸变特征提取 (替代原版复杂的最小二乘法)
    // 坡道的视觉特征：底部的线收缩极快(宽变窄)，上方的线收缩变缓
    
    // 我们取三层采样点计算宽度
    int y_bottom = LOWEST_ROW - 10; // 近端
    int y_mid    = LOWEST_ROW - 30; // 中端
    int y_top    = LOWEST_ROW - 50; // 远端

    int width_bottom = R_boundary[y_bottom] - L_boundary[y_bottom];
    int width_mid    = R_boundary[y_mid] - L_boundary[y_mid];
    int width_top    = R_boundary[y_top] - L_boundary[y_top];

    // 计算两段的收缩量
    int shrink_down = width_bottom - width_mid; // 下段收缩量
    int shrink_up   = width_mid - width_top;    // 上段收缩量

    // 核心判据：如果下段收缩量远大于上段收缩量，说明出现了“折角透视”，即坡道底端
    // 具体比例阈值可根据实际摄像头高度微调 (这里设下段收缩是上段的1.5倍以上)
    if (shrink_down > 15 && shrink_up > 0 && shrink_down > (shrink_up * 1.5f)) {
        return true; 
    }

    return false;
}