#include "edge_extract.h"

TrackState_t track_state = {0}; 

// 图像缓存数组
uint8_t img_gray[TRACK_H][TRACK_W] = {0};
uint8_t img_bin[TRACK_H][TRACK_W] = {0};

// 边界数组
uint8_t L_boundary[TRACK_H] = {0};
uint8_t R_boundary[TRACK_H] = {0};
uint8_t M_boundary[TRACK_H] = {0};

//漫水填充队列 (扩大以防8连通爆队列)
static uint16_t fill_queue[TRACK_W * TRACK_H];

extern void Cross_Detect();
extern void Island_Detect();
extern void Get01change_dajin(); 

//元素检测
// extern bool detect_zebra_crossing(void); // 斑马线
// extern bool Special_Barrier_Detect(void); 
// extern BypassDir_e Get_Special_Barrier_Dir(void); // 获取特殊障碍物绕行方向
// extern bool Barrier_Detect(void);   // 标准障碍物
// extern bool Ramp_Detect(void);      // 坡道

// 蜂鸣器
extern void beep_on(void);          
extern void beep_off(void);


extern IslandController_t IslandCtrl;
extern int Road_width[TRACK_H];
int fill_point_x = TRACK_W / 2;


// 两点连线的最小二乘法斜率
static float Mid_calculateSlopeConnect(int first_y, int second_y, int first_x, int second_x) {
    float sumX = first_y + second_y;
    float sumY = first_x + second_x;
    float sumXY = first_y * first_x + second_y * second_x;
    float sumXX = first_y * first_y + second_y * second_y;
    static float lastValidSlope = 0;
    
    int numPoints = 2;
    float denominator = numPoints * sumXX - sumX * sumX;
    if (fabs(denominator) < 1e-5) return lastValidSlope;
    
    float slope = (numPoints * sumXY - sumX * sumY) / denominator;
    lastValidSlope = slope;
    return slope;
}

// 补线算法：两点之间连直线
static void Mid_Point2Point(int x1, int y1, int x2, int y2) {
    if(x1 > x2) {
        int temp = x1; x1 = x2; x2 = temp;
        temp = y1; y1 = y2; y2 = temp;
    }
    float sumX = x1 + x2;
    float sumY = y1 + y2;
    float meanX = sumX / 2.0f;
    float meanY = sumY / 2.0f;
    
    float slope = Mid_calculateSlopeConnect(x1, x2, y1, y2);
    float intercept = meanY - slope * meanX;
    
    for (int y = x1 + 1; y < x2; ++y) {
        int target_x = (int)(slope * y + intercept + 0.5f); 
        if (target_x < 0) target_x = 0;
        if (target_x > TRACK_W - 1) target_x = TRACK_W - 1;
        M_boundary[y] = target_x;
    }
}

// 计算多点斜率和截距 (用于远端前瞻预测)
static void M_calculateSlopeIntercept(int startY, int endY, uint8_t border[], float* slope, float* intercept) {
    long sumX = 0, sumY = 0;
    long sumXY = 0, sumXX = 0;
    if (startY < 0) startY = 0;
    
    int numPoints = endY - startY + 1;
    if (numPoints <= 0) {
        *slope = 0;
        *intercept = 0;
        return;
    }
    
    for (int y = startY; y <= endY; ++y) {
        sumX += y;
        sumY += border[y];
        sumXY += y * border[y];
        sumXX += y * y;
    }
    
    float meanX = (float)sumX / numPoints;
    float meanY = (float)sumY / numPoints;
    
    float denominator = numPoints * sumXX - sumX * sumX;
    if (fabs(denominator) < 1e-5) {
        *slope = 0;
    } else {
        *slope = (numPoints * sumXY - sumX * sumY) / denominator;
    }
    *intercept = meanY - (*slope) * meanX;
}

//获取一帧图像
int fetch_new_frame(void) {
    track_uvc_get_image(img_gray[0]); 
    int center_x = TRACK_W / 2;
    int center_y = TRACK_H / 2;
    int sum = img_gray[center_y][center_x] + img_gray[center_y + 10][center_x] + img_gray[center_y][center_x + 10];
    if (sum == 0) return 0;
    return 1;
}

static void push_external_black_pixel(int y, int x, int *tail) {
    if (x < 0 || x >= TRACK_W || y < 0 || y >= TRACK_H) return;
    if (img_bin[y][x] != 0) return;

    img_bin[y][x] = 64;
    fill_queue[(*tail)++] = (y << 8) | x;
}

static void fill_internal_black_holes(void) {
    const int dx[] = {0, 0, -1, 1, -1, -1, 1, 1};
    const int dy[] = {-1, 1, 0, 0, -1, 1, -1, 1};
    int head = 0;
    int tail = 0;

    // Mark black areas connected to the image border, then fill the remaining inner holes.
    for (int x = 0; x < TRACK_W; x++) {
        push_external_black_pixel(0, x, &tail);
        push_external_black_pixel(TRACK_H - 1, x, &tail);
    }
    for (int y = 0; y < TRACK_H; y++) {
        push_external_black_pixel(y, 0, &tail);
        push_external_black_pixel(y, TRACK_W - 1, &tail);
    }

    while (head < tail) {
        uint16_t node = fill_queue[head++];
        int y = node >> 8;
        int x = node & 0xFF;

        for (int d = 0; d < 8; d++) {
            push_external_black_pixel(y + dy[d], x + dx[d], &tail);
        }
    }

    for (int y = 0; y < TRACK_H; y++) {
        for (int x = 0; x < TRACK_W; x++) {
            if (img_bin[y][x] == 0) img_bin[y][x] = 255;
            else if (img_bin[y][x] == 64) img_bin[y][x] = 0;
        }
    }
}

void extract_track_region(void) {
    int check = 0;

    if(track_state.special_barrier_dir==DIR_LEFT){
        fill_point_x = TRACK_W - 1;
    }
    if(track_state.special_barrier_dir==DIR_RIGHT){
        fill_point_x = 0;
    }
    
    // 从底层两边分别寻点发散，使用 8连通 漫水填充
    const int dx[] = {0, 0, -1, 1, -1, -1, 1, 1};
    const int dy[] = {-1, 1, 0, 0, -1, 1, -1, 1};

    // 从中心向右寻点填充
    for(int i = fill_point_x; i < TRACK_W; ++i){
        if(check) break;
        if(img_bin[LOWEST_ROW][i] == 255) {
            int head = 0, tail = 0;
            fill_queue[tail++] = (LOWEST_ROW << 8) | i;
            img_bin[LOWEST_ROW][i] = 128; 
            while (head < tail) {
                uint16_t node = fill_queue[head++];
                int y = node >> 8;
                int x = node & 0xFF;
                for (int d = 0; d < 8; d++) {
                    int nx = x + dx[d];
                    int ny = y + dy[d];
                    if (nx >= 0 && nx < TRACK_W && ny >= 0 && ny < TRACK_H) {
                        if (img_bin[ny][nx] == 255) {
                            img_bin[ny][nx] = 128; 
                            fill_queue[tail++] = (ny << 8) | nx; 
                        }
                    }
                }
            }
            check = 1;
        }
    }

    // 从中心向左寻点填充
    for(int i = fill_point_x; i >= 0; --i){
        if(check) break;
        if(img_bin[LOWEST_ROW][i] == 255) {
            int head = 0, tail = 0;
            fill_queue[tail++] = (LOWEST_ROW << 8) | i;
            img_bin[LOWEST_ROW][i] = 128; 
            while (head < tail) {
                uint16_t node = fill_queue[head++];
                int y = node >> 8;
                int x = node & 0xFF;
                for (int d = 0; d < 8; d++) {
                    int nx = x + dx[d];
                    int ny = y + dy[d];
                    if (nx >= 0 && nx < TRACK_W && ny >= 0 && ny < TRACK_H) {
                        if (img_bin[ny][nx] == 255) {
                            img_bin[ny][nx] = 128; 
                            fill_queue[tail++] = (ny << 8) | nx; 
                        }
                    }
                }
            }
            check = 1;
        }
    }

    // 杂点清零
    for (int i = 0; i < TRACK_H; i++) {
        for (int j = 0; j < TRACK_W; j++) {
            if (img_bin[i][j] == 128) img_bin[i][j] = 255;
            else img_bin[i][j] = 0;
        }
    }

    // 寻找最长白列
    track_state.longest_white_len = 0;
    track_state.longest_white_x = 0; 
    for (int x = 0; x < TRACK_W; x++) {
        int current_len = 0;
        for (int y = LOWEST_ROW; y >= 0; y--) {
            if (img_bin[y][x] == 255) current_len++;
            else break; 
        }
        track_state.white_pixel_count[x] = current_len;
        if (current_len > track_state.longest_white_len) {
            track_state.longest_white_len = current_len;
            track_state.longest_white_x = x;
        }
    }
}

void find_track_boundaries(void) {
    for (int i = 0; i < TRACK_H; i++) {
        L_boundary[i] = 0;
        R_boundary[i] = TRACK_W - 1;
    }

    int start_row = LOWEST_ROW;
    int end_row = LOWEST_ROW - track_state.longest_white_len;
    if (end_row < 0) end_row = 0;

    int anchor_x = track_state.longest_white_x; 

    // 从底向上，从锚点向两侧补正边线
    for (int i = start_row; i >= end_row; i--) {
        for (int j = anchor_x; j > 0; j--) {
            if (img_bin[i][j] == 255 && img_bin[i][j - 1] == 0) {
                L_boundary[i] = j - 1;  
                break;                  
            }
        }
        for (int j = anchor_x; j < TRACK_W - 1; j++) {
            if (img_bin[i][j] == 255 && img_bin[i][j + 1] == 0) {
                R_boundary[i] = j + 1;  
                break;                  
            }
        }
    }
}

// 直道均方误差判定 
bool L_Straight_Judge() {
    int start = LOWEST_ROW, end;
    for(int i = LOWEST_ROW; i >= 0; i--) {
        if(L_boundary[i] != 0) {
            start = i; break;
        }
    }
    end = start - 50 > LOWEST_ROW - track_state.longest_white_len ? start - 50 : LOWEST_ROW - track_state.longest_white_len;
    if(start <= end) return 0;
    
    float k = (float)(L_boundary[start] - L_boundary[end]) / (float)(start - end);
    float sumErr = 0;
    for(int i = start; i >= end; i--) {
        float predict = L_boundary[start] + k * (i - start);
        float error = predict - L_boundary[i];
        sumErr += error * error;
    }
    sumErr /= (float)(start - end + 1);
    if(sumErr < 1) return 1;
    return 0; 
}

bool R_Straight_Judge() {
    int start = LOWEST_ROW, end;
    for(int i = LOWEST_ROW; i >= 0; i--) {
        if(R_boundary[i] != TRACK_W - 1) {
            start = i; break;
        }
    }
    end = start - 50 > LOWEST_ROW - track_state.longest_white_len ? start - 50 : LOWEST_ROW - track_state.longest_white_len;
    if(start <= end) return 0;
    
    float k = (float)(R_boundary[start] - R_boundary[end]) / (float)(start - end);
    float sumErr = 0;
    for(int i = start; i >= end; i--) {
        float predict = R_boundary[start] + k * (i - start);
        float error = predict - R_boundary[i];
        sumErr += error * error;
    }
    sumErr /= (float)(start - end + 1);
    if(sumErr < 1) return 1;
    return 0; 
}

void detect_track_elements(void) {
    // ==========================================================
    // 优先级 1：发车斑马线/停止线 (最高级，全局生效)
    // ==========================================================
    // track_state.is_zebra = detect_zebra_crossing();

    // ==========================================================
    // 优先级 2：特殊障碍物 (叠加态 Overlay State)
    // ==========================================================
    // if (Special_Barrier_Detect()) {
    //     track_state.is_special_barrier = true;
    //     track_state.special_barrier_dir = Get_Special_Barrier_Dir();
    // } else {
    //     track_state.is_special_barrier = false;
    //     track_state.special_barrier_dir = DIR_NONE;
    // }
    
    // ==========================================================
    // 优先级 3：标准赛道地形 (基础态 Base State，互斥排他)
    // ==========================================================
    // bool is_base_locked = track_state.is_island || 
    //                       track_state.is_cross || 
    //                       track_state.is_ramp || 
    //                       track_state.is_barrier; 

    //if (!is_base_locked) {
        // if (Barrier_Detect()) {
        //     track_state.is_barrier = true;
        // } 
        // else if (Ramp_Detect()) {
        //     track_state.is_ramp = true;
        // } 
        // else if (Cross_Detect()) {
        //     track_state.is_cross = true;
        // } 
        // else if (Island_Detect()) {
        //     track_state.is_island = true;
        // }
   //}
    if (!track_state.is_island){
        Cross_Detect();
    }
    if (!track_state.is_cross){
        Island_Detect();
    }

    //如果找到元素蜂鸣器就响
    if (track_state.is_island || track_state.is_cross || 
        track_state.is_ramp || track_state.is_barrier || 
        track_state.is_zebra || track_state.is_special_barrier) 
    {
        beep_on();
    } else {
        beep_off();
    }

    // 判断是否是直道 (严格条件)
    if (!track_state.is_island && !track_state.is_ramp && !track_state.is_barrier) {
        if(R_Straight_Judge() && L_Straight_Judge() && track_state.longest_white_len > 60) {
            track_state.is_straight = 1; 
        } else {
            track_state.is_straight = 0; 
        }
    }
}

//计算中线
void calculate_centerline(void) {

    // 1. 基础中线计算
    for (int i = 0; i < TRACK_H; i++) {
        M_boundary[i] = (L_boundary[i] + R_boundary[i]) / 2;
    }

    // 若无干扰元素，执行高级中线修复与预测
    //if (!track_state.is_island && !track_state.is_barrier) {
        int M_start = 0, M_end = 0, P_M = 0, P_S = 0;

        // 入弯单边丢线的断点桥接预测状态机
        for (int i = TRACK_H - 5; i >= 0; --i) {
            if (img_bin[i][M_boundary[i]] == 0) break; 
            
            if (R_boundary[i] != TRACK_W - 1 && L_boundary[i] != 0) P_S = 1; 
            
            if (P_S) {
                if (P_M) {
                    if (R_boundary[i] != TRACK_W - 1 && L_boundary[i] != 0) {
                        M_end = i;
                        P_M = 0;
                        Mid_Point2Point(M_start, M_boundary[M_start], M_end, M_boundary[M_end]);
                    }
                } else {
                    if (R_boundary[i] == TRACK_W - 1 || L_boundary[i] == 0) { 
                        M_start = i + 1;
                        P_M = 1;
                    }
                }
            }
        }

        // 纵向 IIR 级联低通滤波 (确保Y轴平滑)
        for (int i = LOWEST_ROW; i >= LOWEST_ROW - track_state.longest_white_len; --i) {
            if (i + 1 < TRACK_H) {
                M_boundary[i] = M_boundary[i] * 2 / 3 + M_boundary[i + 1] / 3;
            }
        }

        float M_K = 0, M_B = 0; 
        // if(track_state.longest_white_len)
        // {
        //     M_K = ((float)R_boundary[TRACK_H - 1 - track_state.longest_white_len] - 
        //             (float)(Road_width[TRACK_H - 1 - track_state.longest_white_len] / 2) - (float)(TRACK_W - 1) / 2) / 
        //                 ((float)(TRACK_H - 1 - track_state.longest_white_len) - (float)(TRACK_H - 1)) ;
        //     M_B = (float)(TRACK_W - 1) / 2 - M_K * (float)(TRACK_H - 1);
            
        //     if(IslandCtrl.state == ISLAND_STATE_FINDEXIT_V || IslandCtrl.state == ISLAND_STATE_EXITING)
        //     {
        //         for (int i = LOWEST_ROW - track_state.longest_white_len; i < TRACK_H; ++i) {
        //             int x = (int)(M_K * i + M_B + 0.5f);
        //             if (x < 0) x = 0;
        //             if (x > TRACK_W - 1) x = TRACK_W - 1;
        //             M_boundary[i] = x;
        //         }
        //         printf("%f,%f\r\n",(float)R_boundary[TRACK_H - 1 - track_state.longest_white_len] - (float)(Road_width[TRACK_H - 1 - track_state.longest_white_len] / 2), TRACK_H - 1 - track_state.longest_white_len);
        //         printf("L_W:%d, R:%d\r\n",track_state.longest_white_len, R_boundary[TRACK_H - 1 - track_state.longest_white_len]);
        //     }
        // }
        
        //fill_point_x = TRACK_W / 2;

        if(track_state.special_barrier_dir==DIR_LEFT){
            for (int i = LOWEST_ROW - track_state.longest_white_len - 1; i < TRACK_H; i++) {
                M_boundary[i] = L_boundary[i];
            }
        }
        if(track_state.special_barrier_dir==DIR_RIGHT){
            for (int i = LOWEST_ROW - track_state.longest_white_len - 1; i < TRACK_H; i++) {
                M_boundary[i] = R_boundary[i];
            }
        }

        // 远端盲区最小二乘法前瞻延伸
        M_K = 0;
        M_B = 0; 
        M_calculateSlopeIntercept(LOWEST_ROW - track_state.longest_white_len, LOWEST_ROW, M_boundary, &M_K, &M_B); 
        
        for (int i = LOWEST_ROW - track_state.longest_white_len - 1; i >= 0; --i) {
            int x = (int)(M_K * i + M_B + 0.5f);
            if (x < 0) x = 0;
            if (x > TRACK_W - 1) x = TRACK_W - 1;
            M_boundary[i] = x;
        }
    //}

    //绕标志牌
    // if(track_state.special_barrier_dir==DIR_LEFT&&track_state.is_special_barrier){
    //     for (int i = 0; i < TRACK_H; i++) {
    //         M_boundary[i] = M_boundary[i]-30;
    //     }
    // }
    // if(track_state.special_barrier_dir==DIR_RIGHT&&track_state.is_special_barrier){
    //     for (int i = 0; i < TRACK_H; i++) {
    //         M_boundary[i] = M_boundary[i]+30;
    //     }
    // }
    //test

    

}

// 越界停车保护机制 
// void check_failsafe(void) {
//     bool safe = false;
//     for(int i = 0; i < TRACK_W; i++) {
//         if(img_bin[LOWEST_ROW][i] == 255) {
//             safe = true;
//             break;
//         }
//     }

//     if (!safe) {
//         track_state.stop_num++; 
//     } else {
//         track_state.stop_num = 0; 
//         // if (track_state.stop_flag == 2) {
//         //     track_state.stop_flag = 0; 
//         // }
//     }
    
//     if (track_state.stop_num > 5) {
//         track_state.stop_flag = 1; 
//     }
// }

void edge_extract(void) {
    if (fetch_new_frame() == 0) return; 
    Get01change_dajin();
    extract_track_region();
    //fill_internal_black_holes();
    find_track_boundaries();
    detect_track_elements();
    
    calculate_centerline();
    //check_failsafe(); // Failsafe是基础巡线保底，启用执行

}