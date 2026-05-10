#include "cross.h"

// ==========================================================
// 辅助函数：两点直接连线修补
// ==========================================================
static void patch_line_2pts(uint8_t *boundary, int y_up, int x_up, int y_down, int x_down) 
{
    if (y_up >= y_down || y_up < 0 || y_down >= TRACK_H) return;
    
    // 计算斜率的倒数 (dx / dy)
    float inv_k = (float)(x_down - x_up) / (y_down - y_up);
    
    for (int y = y_up + 1; y < y_down; y++) {
        int x = x_up + (int)((y - y_up) * inv_k);
        // 限幅保护
        if (x < 0) x = 0;
        if (x >= TRACK_W) x = TRACK_W - 1;
        boundary[y] = x;
    }
}

// ==========================================================
// 辅助函数：单点向下延长修补 (利用角点上方5个点的趋势向下延伸)
// ==========================================================
static void patch_line_extend(uint8_t *boundary, int y_up, int x_up) 
{
    if (y_up - 5 < 0) return; // 上方点不够，无法计算趋势
    
    // 简易斜率计算：取角点上方第5个点与角点求斜率
    int x_top = boundary[y_up - 5];
    float inv_k = (float)(x_up - x_top) / 5.0f;
    
    for (int y = y_up + 1; y < TRACK_H; y++) {
        int x = x_up + (int)((y - y_up) * inv_k);
        if (x < 0) x = 0;
        if (x >= TRACK_W) x = TRACK_W - 1;
        boundary[y] = x;
    }
}

// 查找左上的角点 (物理上是十字路口的远端左角)
static CornerPoint_t find_left_up_corner(int start_y, int end_y) {
    CornerPoint_t pt = {0, 0, false};
    for (int i = start_y; i >= end_y + 4; i--) {
        // 角点特征：上方连续平缓，下方突然剧烈丢线向左(X归零)
        if (L_boundary[i] - L_boundary[i+2] >= 8 && 
            L_boundary[i] - L_boundary[i+4] >= 15) 
        {
            pt.y = i;
            pt.x = L_boundary[i];
            pt.found = true;
            break;
        }
    }
    return pt;
}

// 查找右上的角点
static CornerPoint_t find_right_up_corner(int start_y, int end_y) {
    CornerPoint_t pt = {0, 0, false};
    for (int i = start_y; i >= end_y + 4; i--) {
        // 角点特征：上方连续平缓，下方突然剧烈丢线向右(X跑到最大值)
        if (R_boundary[i+2] - R_boundary[i] >= 8 && 
            R_boundary[i+4] - R_boundary[i] >= 15) 
        {
            pt.y = i;
            pt.x = R_boundary[i];
            pt.found = true;
            break;
        }
    }
    return pt;
}

// 查找左下的角点 (近端左角)
static CornerPoint_t find_left_down_corner(int start_y, int end_y) {
    CornerPoint_t pt = {0, 0, false};
    for (int i = start_y; i >= end_y; i--) {
        if (i - 4 < 0) break;
        // 角点特征：下方连续平缓，上方突然剧烈丢线
        if (L_boundary[i] - L_boundary[i-2] >= 8 && 
            L_boundary[i] - L_boundary[i-4] >= 15) 
        {
            pt.y = i;
            pt.x = L_boundary[i];
            pt.found = true;
            break;
        }
    }
    return pt;
}

// 查找右下的角点
static CornerPoint_t find_right_down_corner(int start_y, int end_y) {
    CornerPoint_t pt = {0, 0, false};
    for (int i = start_y; i >= end_y; i--) {
        if (i - 4 < 0) break;
        if (R_boundary[i-2] - R_boundary[i] >= 8 && 
            R_boundary[i-4] - R_boundary[i] >= 15) 
        {
            pt.y = i;
            pt.x = R_boundary[i];
            pt.found = true;
            break;
        }
    }
    return pt;
}

bool Cross_Detect(void)
{
    int both_lost_time = 0;
    int search_end = LOWEST_ROW - track_state.longest_white_len;
    if (search_end < 5) search_end = 5;

    // 1. 初筛：检查是否有明显的“双边丢线”特征 (赛道变宽)
    for (int i = LOWEST_ROW; i >= search_end; i--) {
        if (L_boundary[i] == 0 && R_boundary[i] == TRACK_W - 1) {
            both_lost_time++;
        }
    }
    
    // 如果丢线少于 10 行，绝对不可能是十字，直接退出，节省算力
    if (both_lost_time < 10) {
        return false;
    }

    // 2. 寻找上半部分角点 (十字路口远端)
    CornerPoint_t L_Up = find_left_up_corner(LOWEST_ROW - 5, search_end);
    CornerPoint_t R_Up = find_right_up_corner(LOWEST_ROW - 5, search_end);

    // 十字路口必须同时具备左右两个上角点，否则可能是弯道丢线
    if (!L_Up.found || !R_Up.found) {
        return false;
    }

    // ----------------------------------------------------
    // 判定成功：走到这里，说明 100% 确认前方是十字路口！
    // 接下来执行边界修补，防止车子在十字中心乱扭
    // ----------------------------------------------------

    // 3. 寻找下半部分角点 (十字路口近端)
    // 搜索上限是从上角点往下，不用搜整张图
    int down_search_start = (L_Up.y > R_Up.y) ? L_Up.y : R_Up.y; 
    
    CornerPoint_t L_Down = find_left_down_corner(LOWEST_ROW - 6, down_search_start + 2);
    CornerPoint_t R_Down = find_right_down_corner(LOWEST_ROW - 6, down_search_start + 2);

    // 逻辑保护：下角点在画面中的 Y 值，必须比上角点大（更靠下）
    if (L_Down.found && L_Down.y <= L_Up.y) L_Down.found = false;
    if (R_Down.found && R_Down.y <= R_Up.y) R_Down.found = false;

    // 4. 交叉路口补线策略 (四种组合)
    // 情况 A：左边找到下点 (1)，补线
    if (L_Down.found) {
        patch_line_2pts(L_boundary, L_Up.y, L_Up.x, L_Down.y, L_Down.x);
    } else { // 情况 B：没找到下点 (0)，靠上方趋势向下延长
        patch_line_extend(L_boundary, L_Up.y, L_Up.x);
    }

    // 情况 C：右边找到下点 (1)，补线
    if (R_Down.found) {
        patch_line_2pts(R_boundary, R_Up.y, R_Up.x, R_Down.y, R_Down.x);
    } else { // 情况 D：没找到下点 (0)，靠上方趋势向下延长
        patch_line_extend(R_boundary, R_Up.y, R_Up.x);
    }

    return true; 
}