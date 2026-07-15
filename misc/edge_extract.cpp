#include "edge_extract.h"

TrackState_t track_state = {0}; 

// 图像缓存数组
uint8_t img_gray[TRACK_H][TRACK_W] = {0};
uint8_t img_bin[TRACK_H][TRACK_W] = {0};

// 边界数组
uint8_t L_boundary[TRACK_H] = {0};
uint8_t R_boundary[TRACK_H] = {0};
uint8_t M_boundary[TRACK_H] = {0};
vector<pair<uint8_t, uint8_t>> L_boundary_8;
vector<pair<uint8_t, uint8_t>> R_boundary_8;
vector<Point> fact_L_boundary, fact_R_boundary;


extern void Cross_Detect();
extern void Island_Detect();
extern void Get01change_dajin(); 

// 蜂鸣器
extern void beep_on(void);          
extern void beep_off(void);

// 方向枚举定义
enum Direction {
    LEFT_DOWN = 0,     // 0: (0,1)
    LEFT,               // 1: (-1,1)
    LEFT_UP,            // 2: (-1,0)
    UP_LEFT,            // 3: (-1,-1)
    UP,                 // 4: (0,-1)
    RIGHT_UP,           // 5: (1,-1)
    RIGHT,              // 6: (1,0)
    DOWN_RIGHT          // 7: (1,1)
};

struct BorderConfig {
    int borderMin = 0;            // 允许从0边缘开始搜
    int borderMax = TRACK_W - 1;  // 允许贴到最右侧边缘
    int whitePixel = 255;
    int blackPixel = 0;
    int maxIterations = 5000;
    int morphologySize = 3;
    int stabilityCheckDepth = 3;  // 连续同点检测深度
};
BorderConfig config;

//八邻域寻线
pair<vector<Point>, vector<Point>> trackEdges(
    const Mat& binImage, 
    Point leftStart, 
    Point rightStart,
    const BorderConfig& config) 
{
    // 搜索方向模板
    const array<Point, 8> leftSearchOrder = {
        Point(0, 1),   // 0: 向下
        Point(-1, 1),  // 1: 左下 
        Point(-1, 0),  // 2: 左
        Point(-1, -1), // 3: 左上
        Point(0, -1),  // 4: 上
        Point(1, -1),  // 5: 右上
        Point(1, 0),   // 6: 右
        Point(1, 1)    // 7: 右下
    };

    const array<Point, 8> rightSearchOrder = {
        Point(0, 1),   // 0: 向下
        Point(1, 1),   // 1: 右下 
        Point(1, 0),   // 2: 右
        Point(1, -1),  // 3: 右上
        Point(0, -1),  // 4: 上
        Point(-1, -1), // 5: 左上
        Point(-1, 0),  // 6: 左
        Point(-1, 1)   // 7: 左下
    };

    vector<Point> leftPoints, rightPoints;
    vector<int> leftDirections, rightDirections;
    Point currentLeft = leftStart;
    Point currentRight = rightStart;
    int safeCounter = 0;

    int leftLastDir = -1;
    int rightLastDir = -1;

    // 安全获取像素，将图像越界外围统统视为黑色（虚拟黑墙）
    auto getPixelSafe = [&](int x, int y) -> uint8_t {
        if (x < 0 || x >= binImage.cols || y < 0 || y >= binImage.rows) {
            return config.blackPixel; 
        }
        return binImage.at<uint8_t>(y, x);
    };

    auto findNextPoint = [&](const Point& current, 
                             const array<Point, 8>& order,
                             int lastDir) -> pair<Point, int> 
    {
        int startIdx = (lastDir == -1) ? 0 : (lastDir + 5) % 8;

        for (int step = 0; step < 8; ++step) {
            int i = (startIdx + step) % 8;
            const Point neighbor = current + order[i];
            const Point nextNeighbor = current + order[(i + 1) % 8];
            
            // 采用虚拟黑墙读取像素
            uint8_t valNeighbor = getPixelSafe(neighbor.x, neighbor.y);
            uint8_t valNextNeighbor = getPixelSafe(nextNeighbor.x, nextNeighbor.y);

            // 如果 neighbor 是黑（或者是屏幕外），且 nextNeighbor 是白
            if (valNeighbor == config.blackPixel && valNextNeighbor == config.whitePixel) {
                // 能读出白色，说明 nextNeighbor 一定在屏幕内部，无需再做边界拦截
                return {nextNeighbor, (i + 1) % 8};
            }
        }
        return {current, -1};
    };

    bool leftStagnant = false;
    bool rightStagnant = false;

    while (safeCounter++ < config.maxIterations) {
        bool leftUpdated = false;
        bool rightUpdated = false;

        // ================= 【核心：等高同步控制逻辑】 =================
        // y坐标越大，代表在图像中越靠下（越靠近车身），说明该侧落后了。
        bool allowLeft = true;
        bool allowRight = true;

        if (!leftStagnant && !rightStagnant) {
            if (currentLeft.y > currentRight.y) {
                allowRight = false; // 左边落后，右边暂停等待
            } else if (currentRight.y > currentLeft.y) {
                allowLeft = false;  // 右边落后，左边暂停等待
            }
            // 如果 currentLeft.y == currentRight.y，两边 allow 都为 true，一起走
        }

        // ================= 左边处理 =================
        if (allowLeft && !leftStagnant && currentLeft.x >= 0 && currentLeft.y >= 0) {
            auto [newLeft, dir] = findNextPoint(currentLeft, leftSearchOrder, leftLastDir);
            if (dir != -1) {
                // 方向突变检测
                if (leftDirections.size() >= 3) {
                    int dirChange = abs(dir - leftDirections.back());
                    if (dirChange > 4) dirChange = 8 - dirChange;
                    if (dirChange > 2) leftStagnant = true; // 发生锐角转折
                }

                if (!leftStagnant) {
                    leftPoints.push_back(newLeft);
                    leftDirections.push_back(dir);
                    currentLeft = newLeft;
                    leftLastDir = dir;
                    leftUpdated = true;
                }
            } else {
                leftStagnant = true; 
            }
        }

        // ================= 右边处理 =================
        if (allowRight && !rightStagnant && currentRight.x >= 0 && currentRight.y >= 0) {
            auto [newRight, dir] = findNextPoint(currentRight, rightSearchOrder, rightLastDir);
            if (dir != -1) {
                // 方向突变检测
                if (rightDirections.size() >= 3) {
                    int dirChange = abs(dir - rightDirections.back());
                    if (dirChange > 4) dirChange = 8 - dirChange;
                    if (dirChange > 2) rightStagnant = true; // 发生锐角转折
                }

                if (!rightStagnant) {
                    rightPoints.push_back(newRight);
                    rightDirections.push_back(dir);
                    currentRight = newRight;
                    rightLastDir = dir; 
                    rightUpdated = true;
                }
            } else {
                rightStagnant = true; 
            }
        }

        // ================= 稳定性检查 =================
        if (leftPoints.size() >= static_cast<std::size_t>(config.stabilityCheckDepth)) {
            auto& p1 = leftPoints.end()[-1];
            auto& p2 = leftPoints.end()[-2];
            auto& p3 = leftPoints.end()[-3];
            if ((p1 == p2) && (p2 == p3)) leftStagnant = true;
        }
        
        if (rightPoints.size() >= static_cast<std::size_t>(config.stabilityCheckDepth)) {
            auto& p1 = rightPoints.end()[-1];
            auto& p2 = rightPoints.end()[-2];
            auto& p3 = rightPoints.end()[-3];
            if ((p1 == p2) && (p2 == p3)) rightStagnant = true;
        }

        // 边界相遇检测
        bool boundariesMeet = false;
        if (!leftPoints.empty() && !rightPoints.empty()) {
            Point lastLeft = leftPoints.back();
            Point lastRight = rightPoints.back();
            boundariesMeet = (abs(lastLeft.x - lastRight.x) < 2) && 
                             (abs(lastLeft.y - lastRight.y) < 2);
        }

        // (注：这里删除了你原来强行 pop_back 的 "下移等待机制" 代码，
        // 因为上面的 allowLeft/allowRight 已经优雅地实现了严格等高，不再需要删点了)

        // ================= 终止条件 =================
        if ((leftStagnant && rightStagnant) || boundariesMeet || (!leftUpdated && !rightUpdated)) {
            break;
        }
    }

    return {leftPoints, rightPoints};
}

void extract_track_region(void) {
    Mat binary(TRACK_H, TRACK_W, CV_8U, &img_bin[0][0]);
    config.borderMin = 0;
    config.borderMax = binary.cols - 1;

    L_boundary_8.clear();
    R_boundary_8.clear();

    for (int i = 0; i < TRACK_H; i++) {
        L_boundary[i] = 0;
        R_boundary[i] = TRACK_W - 1;
    }

    // 起点检测：同样引入虚拟黑墙机制来判断最底部的点是否贴边
    auto [foundLeft, foundRight, leftStart, rightStart] = [&](){
        const int startRow = binary.rows - 2;
        Point left(-1, startRow), right(-1, startRow);
        bool bLeft = false, bRight = false;
        uint8_t R_Start = TRACK_W - 1;
        uint8_t L_Start = 1;
        for(int i = R_Start; i >= 0; i--)
        {
            if(img_bin[startRow][i] == 255)
            {
                R_Start = i;
                break;
            }
        }

        for(int i = L_Start; i < TRACK_W; i++)
        {
            if(img_bin[startRow][i] == 255)
            {
                L_Start = i;
                break;
            }
        }

        //以（L_Start，startRow）为坐标原点进行洪水填充

        Mat mask = Mat::zeros(TRACK_H+2, TRACK_W+2, CV_8U);// 掩码图像
        floodFill(binary, mask, Point(L_Start,startRow), Scalar(255), 0, Scalar(), Scalar(), 8 | (255 << 8));// 填充连通域
        for(int y=0; y<TRACK_H; ++y){// 遍历掩码图像
            uchar* m = mask.ptr(y+1)+1;//获取掩码图像的指针
            for(int x=0; x<TRACK_W; ++x){// 遍历掩码图像的每一个像素
                if(m[x] == 0) img_bin[y][x] = 0;// 如果掩码图像的像素为0，则将原图像的像素置为0
            }
        }

        // 左边搜索
        for (int x = L_Start; x >= config.borderMin; --x) {
            uint8_t p = binary.at<uint8_t>(startRow, x);
            uint8_t left_p = (x - 1 < 0) ? config.blackPixel : binary.at<uint8_t>(startRow, x-1);
            if (p == config.whitePixel && left_p == config.blackPixel) {
                left = Point(x, startRow);
                bLeft = true;
                break;
            }
        }

        // 右边搜索
        for (int x = R_Start; x <= config.borderMax; ++x) {
            uint8_t p = binary.at<uint8_t>(startRow, x);
            uint8_t right_p = (x + 1 >= binary.cols) ? config.blackPixel : binary.at<uint8_t>(startRow, x+1);
            if (p == config.whitePixel && right_p == config.blackPixel) {
                right = Point(x, startRow);
                bRight = true;
                break;
            }
        }
        return make_tuple(bLeft, bRight, left, right);
    }();

    if (foundLeft || foundRight) {

        auto [leftPoints, rightPoints] = trackEdges(binary, leftStart, rightStart, config);

        fact_L_boundary = leftPoints;
        fact_R_boundary = rightPoints;
        
        for (const auto& p : leftPoints) {
            L_boundary_8.push_back({p.x, p.y});
            if (p.y >= 0 && p.y < TRACK_H) 
            {
                L_boundary[p.y] = fmax(L_boundary[p.y], p.x);
                track_state.longest_white_len = fmin(p.y, track_state.longest_white_len);
            }
        }
        for (const auto& p : rightPoints) {
            R_boundary_8.push_back({p.x, p.y});
            if (p.y >= 0 && p.y < TRACK_H) 
            {
                R_boundary[p.y] = fmin(R_boundary[p.y], p.x);
                track_state.longest_white_len = fmin(p.y, track_state.longest_white_len);
            }
        }
    }

    track_state.longest_white_len = fmin(fmax(0,TRACK_H - track_state.longest_white_len),TRACK_H);

    //printf("%d\r\n",track_state.longest_white_len);

}

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
    if (!track_state.is_island && !track_state.is_barrier) {
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

        // 远端盲区最小二乘法前瞻延伸
        float M_K = 0, M_B = 0; 
        M_calculateSlopeIntercept(LOWEST_ROW - track_state.longest_white_len, LOWEST_ROW-20, M_boundary, &M_K, &M_B); 
        
        for (int i = LOWEST_ROW - track_state.longest_white_len - 1; i >= 0; --i) {
            int x = (int)(M_K * i + M_B + 0.5f);
            if (x < 0) x = 0;
            if (x > TRACK_W - 1) x = TRACK_W - 1;
            M_boundary[i] = x;
        }
    }
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
    detect_track_elements();
    
    // if (track_state.is_special_barrier && track_state.special_barrier_dir != DIR_STRAIGHT) { 
    //     draw_virtual_wall(); // 涂黑半边屏幕
    //     find_track_boundaries(); // 重新扫一次线！
    // }
    
    calculate_centerline();
    //check_failsafe(); // Failsafe是基础巡线保底，启用执行
}