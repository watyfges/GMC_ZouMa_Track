#include "island.h"//环岛检测

#define PI 3.1415926535897932384626433832795028841971693993751058209749445923078164062862089986280348253421170679

using namespace std;

extern uint8 L_boundary[TRACK_H], M_boundary[TRACK_H], R_boundary[TRACK_H]; //左右边界
extern uint8 img_bin[TRACK_H][TRACK_W];//二值化的图像
int Right_Mutation_Point, Left_Mutation_Point; //左右突变点
int Right_Inflection_Point, Left_Inflection_Point; //左右拐点
int Right_V_Point, Left_V_Point; //左右V点
int Right_exit_V_Point,Left_exit_V_Point;//出环岛时的左右V点
int Right_Island_Left_Inflection_Ponit, Left_Island_Right_Inflection_Ponit; //左右环岛拐点
bool check1 = 0; //入环标志位（防止丢点识别不到进岛状态）
int Island_Time = 16; //环岛时间
IslandController_t IslandCtrl = {ISLAND_STATE_NONE, NONE_ISLAND};

//处理完成的标志
//extern uint8 White_Column[UVC_WIDTH],Longest_White_Column, Longest_White_Column_X; //最长白线 ok
//int Island_Flag_State; //环岛标志位ok
//char Island_Direction = 'N'; //环岛方向ok
//int Island_State = 0; //环岛状态ok
//extern uint8 image_b[UVC_HEIGHT][UVC_WIDTH]; //图像ok

//没用的
//int Road_width[TRACK_H]; //道路宽度
//extern std::vector<cv::Point> fact_L_boundary, fact_R_boundary; //实际左右边界
//int White_Inflection_Y, White_Inflection_X; //白线拐点
//int Right_Monotonic_Num, Left_Monotonic_Num; //左右单调段数
//int V_distance = 10;            //V点距离，目前硬编在代码里
//int EX_Island = UVC_HEIGHT * 2 / 3; //环岛出口
//char Longest_White_Column_State = 'N'; //最长白线状态
//int V_set = 0; //V点设置
//int White_Num = 0, Black_Num = 0; //白黑线段数
//int Old_Monotonic = 0; //上一次单调

int curve_width[TRACK_H] = 
    {
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   // 行 000 - 009
    0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   // 行 010 - 019
    0,   0,   0,   0,   0,   2,   5,   9,   13,  15,  // 行 020 - 029
    18,  20,  24,  26,  28,  30,  32,  34,  37,  38,  // 行 030 - 039
    41,  44,  46,  48,  50,  53,  55,  57,  60,  61,  // 行 040 - 049
    64,  66,  68,  71,  72,  75,  78,  80,  82,  85,  // 行 050 - 059
    86,  89,  91,  93,  97,  99,  101, 104, 106, 110, // 行 060 - 069
    111, 114, 118, 119, 122, 124, 126, 129, 130, 133, // 行 070 - 079
    136, 138, 140, 142, 144, 147, 149, 151, 153, 155, // 行 080 - 089
    155, 155, 155, 155, 155, 155, 155, 155, 155, 155, // 行 090 - 099
    155, 155, 155, 155, 155, 155, 155, 155, 155, 155, // 行 100 - 109
    155, 155, 155, 155, 155, 155, 155, 155, 155, 155  // 行 110 - 119
    };

int Road_width[TRACK_H] = 
    {
    11, 12, 12, 13, 15, 16, 16, 18, 20, 20,  // 行 000 - 009
    22, 22, 24, 25, 26, 28, 28, 30, 31, 32,  // 行 010 - 019
    34, 34, 36, 37, 38, 39, 40, 42, 43, 44,  // 行 020 - 029
    46, 46, 47, 49, 50, 51, 53, 53, 55, 55,  // 行 030 - 039
    57, 59, 60, 61, 62, 64, 65, 66, 68, 68,  // 行 040 - 049
    70, 71, 72, 74, 74, 76, 78, 79, 80, 82,  // 行 050 - 059
    82, 84, 85, 86, 88, 89, 90, 92, 93, 95,  // 行 060 - 069
    95, 97, 99, 99, 101, 102, 103, 105, 105, 107,  // 行 070 - 079
    109, 110, 111, 112, 113, 115, 116, 117, 118, 120,  // 行 080 - 089
    121, 122, 124, 124, 126, 126, 128, 130, 130, 132,  // 行 090 - 099
    132, 134, 135, 136, 137, 138, 139, 141, 141, 143,  // 行 100 - 109
    143, 145, 145, 147, 147, 149, 149, 151, 151, 153 // 行 110 - 119
    };

//声明
bool Check_Lost_L();//检查左突变点是否丢失
bool Check_Lost_R();//检查右突变点是否丢失

// 最小二乘法计算斜率连线
float island_calculateSlopeConnect(int first_y, int second_y, int first_x, int second_x) { //最小二乘法计算斜率连线
    float sumX = 0, sumY = 0, sumXY = 0, sumXX = 0; // 统计区间数据
    static float lastValidSlope = 0; // 保留上次有效斜率

    sumX = first_y + second_y;//统计区间数据
    sumY = first_x + second_x;//统计区间数据
    sumXY = first_y * first_x + second_y * second_x;//统计区间数据
    sumXX = first_y * first_y + second_y * second_y;//统计区间数据

    int numPoints = 2;
    float denominator = numPoints * sumXX - sumX * sumX;//计算分母

    // 防止除以零
    if (fabs(denominator) < 1e-5) return lastValidSlope; // 返回保留值

    float slope = (numPoints * sumXY - sumX * sumY) / denominator; //计算斜率
    lastValidSlope = slope; // 更新保留值
    return slope; //返回斜率
}

pair<float, float> Right_Point2Point (int x1,int y1, int x2, int y2){ //右边界两点连线
    if(x1 > x2)//确保x1在x2左侧
    {
        swap(x1,x2);
        swap(y1,y2);
    }
    //统计区间数据
    float sumX = x1 + x2, sumY = y1 + y2;
    // 计算均值
    float meanX = static_cast<float>(sumX) / 2;
    float meanY = static_cast<float>(sumY) / 2;
    // 获取斜率

    float slope = island_calculateSlopeConnect(x1, x2, y1, y2);//计算斜率

    float intercept = meanY - slope * meanX;//计算截距

    for (int y = x1 + 1; y < x2; ++y) {//计算右边界
            int x = cvRound(slope * y + intercept);
            R_boundary[y] = fmin(fmax(x, 0), TRACK_W - 1);
    }

    return {slope, intercept}; //返回斜率与截距
}

pair<float, float> Left_Point2Point (int x1,int y1, int x2, int y2){ //左边界两点连线
    if(x1 > x2)//确保x1在x2左侧
    {
        swap(x1,x2);
        swap(y1,y2);
    }
    //统计区间数据
    float sumX = x1 + x2, sumY = y1 + y2;
    // 计算均值
    float meanX = static_cast<float>(sumX) / 2;
    float meanY = static_cast<float>(sumY) / 2;
    // 获取斜率

    float slope = island_calculateSlopeConnect(x1, x2, y1, y2);//计算斜率

    float intercept = meanY - slope * meanX;//计算截距

    for (int y = x1 + 1; y < x2; ++y) {//计算左边界
            int x = cvRound(slope * y + intercept);
            L_boundary[y] = fmin(fmax(x, 0), TRACK_W - 1);
    }
    return {slope, intercept}; //返回斜率与截距
}

bool Find_EXIT_V_Point(int start, int end) {  //查找上边界的角点
    if (start < end) { //确保start大于end
        swap(start, end);
    }

    // 边界安全限制
    if (start > LOWEST_ROW - 5) { //确保start不小于最低行-5
        start = LOWEST_ROW - 5;
    }
    if (end < LOWEST_ROW - 5 - track_state.longest_white_len) { //确保end不小于最低行-5-最长白色列数
        end = LOWEST_ROW - 5 - track_state.longest_white_len;
    }
    end = fmax(5, end); // 保证 i-3 最小为 2，防止数组越界

    // 从下往上扫
    for (int i = start; i >= end; i--) {
        
        // 找左岛的左侧V点
        if (IslandCtrl.dir == LEFT_ISLAND) {
            if (abs((int)L_boundary[i] - (int)L_boundary[i-1]) <= 5 &&  // 上方贴合直线
                abs((int)L_boundary[i-1] - (int)L_boundary[i-2]) <= 5 &&
                abs((int)L_boundary[i-2] - (int)L_boundary[i-3]) <= 5 &&
                ((int)L_boundary[i] - (int)L_boundary[i+2]) >= 8 &&     // 下方发生撕裂（变宽）
                ((int)L_boundary[i] - (int)L_boundary[i+3]) >= 15 &&
                ((int)L_boundary[i] - (int)L_boundary[i+4]) >= 15) {
                
                Left_exit_V_Point = i; // 记录行数
                return 1;              // 找到后直接返回
            }
        }
        
        // 找右岛的右侧V点
        else if (IslandCtrl.dir == RIGHT_ISLAND) {
            if (abs((int)R_boundary[i] - (int)R_boundary[i-1]) <= 5 &&  // 上方贴合直线
                abs((int)R_boundary[i-1] - (int)R_boundary[i-2]) <= 5 &&
                abs((int)R_boundary[i-2] - (int)R_boundary[i-3]) <= 5 &&
                ((int)R_boundary[i] - (int)R_boundary[i+2]) <= -8 &&    // 下方发生撕裂（变宽）
                ((int)R_boundary[i] - (int)R_boundary[i+3]) <= -15 &&
                ((int)R_boundary[i] - (int)R_boundary[i+4]) <= -15) {
                
                Right_exit_V_Point = i; // 记录行数
                return 1;               // 找到后直接返回
            }
        }
    }
    // 循环结束也没找到符合条件的点
    return 0;
}

bool Right_Monotonic_Mutation(int start, int end) //右突变点判断
{
    if(start < end)swap(start, end);//确保start在end左侧

    if(end < 10)end = 10;//确保end在10以上
    if(start > 70 )start = 70;//确保start在70以下
    
    for(int i=start;i>=end;i--)//会读取前5后5数据，所以前面对输入范围有要求
    {
        if(
        R_boundary[i]<R_boundary[i+5]&&R_boundary[i]<R_boundary[i-5]&&
        R_boundary[i]<R_boundary[i+4]&&R_boundary[i]<R_boundary[i-4]&&
        R_boundary[i]<=R_boundary[i+3]&&R_boundary[i]<=R_boundary[i-3]&&
        R_boundary[i]<=R_boundary[i+2]&&R_boundary[i]<=R_boundary[i-2]&&
        R_boundary[i]<=R_boundary[i+1]&&R_boundary[i]<=R_boundary[i-1]&&
        R_boundary[i + 5] != TRACK_W - 1 && R_boundary[i - 5] != TRACK_W - 1)
        {//就很暴力，这个数据是在前5，后5中最大的，那就是单调突变点
            Right_Mutation_Point=i;//存储突变点
            return 1;
        }
    }

    return 0;
}

bool Left_Monotonic_Mutation(int start, int end) //左突变点判断
{
    if(start < end)swap(start, end);//确保start在end左侧

    if(end < 10)end = 10;//确保end在10以上
    if(start > 70 )start = 70;//确保start在70以下
    
    for(int i=start;i>=end;i--)//会读取前5后5数据，所以前面对输入范围有要求
    {
        if(
        L_boundary[i]>L_boundary[i+5]&&L_boundary[i]>L_boundary[i-5]&&
        L_boundary[i]>L_boundary[i+4]&&L_boundary[i]>L_boundary[i-4]&&
        L_boundary[i]>=L_boundary[i+3]&&L_boundary[i]>=L_boundary[i-3]&&
        L_boundary[i]>=L_boundary[i+2]&&L_boundary[i]>=L_boundary[i-2]&&
        L_boundary[i]>=L_boundary[i+1]&&L_boundary[i]>=L_boundary[i-1]&&
        L_boundary[i + 5] != 0 && L_boundary[i - 5] != 0 )
        {//就很暴力，这个数据是在前5，后5中最大的，那就是单调突变点
            Left_Mutation_Point=i;//存储突变点
            return 1;
        }
    }

    return 0;
}

bool Right_Inflection()//右突变点判断
{
    for(int i = LOWEST_ROW; i > fmax(LOWEST_ROW - track_state.longest_white_len, 20); i--){
        if(//只找第一个符合条件的点
           abs((int)R_boundary[i] - (int)R_boundary[i+1]) <= 5 &&//角点的阈值可以更改
           abs((int)R_boundary[i+1] - (int)R_boundary[i+2]) <= 5 &&
           abs((int)R_boundary[i+2] - (int)R_boundary[i+3]) <= 5 &&
              ((int)R_boundary[i] - (int)R_boundary[i-2]) <= -5 &&
              ((int)R_boundary[i] - (int)R_boundary[i-3]) <= -10 &&
              ((int)R_boundary[i] - (int)R_boundary[i-4]) <= -15 ){
            Right_Inflection_Point = i;//存储突变点
            return 1;
        }
    }

    return 0;
}

bool Left_Inflection()//左突变点判断
{
    for(int i = LOWEST_ROW; i > fmax(LOWEST_ROW - track_state.longest_white_len, 20); i--){
        if(//只找第一个符合条件的点
           abs((int)L_boundary[i] - (int)L_boundary[i+1]) <= 5 &&//角点的阈值可以更改
           abs((int)L_boundary[i+1] - (int)L_boundary[i+2]) <= 5 &&
           abs((int)L_boundary[i+2] - (int)L_boundary[i+3]) <= 5 &&
              ((int)L_boundary[i] - (int)L_boundary[i-2]) >= 5 &&
              ((int)L_boundary[i] - (int)L_boundary[i-3]) >= 10 &&
              ((int)L_boundary[i] - (int)L_boundary[i-4]) >= 15 ){
            Left_Inflection_Point = i;//存储突变点
            return 1;
        }
    }

    return 0;
}

bool Right_Island_Left_Inflection()//右岛屿左突变点判断
{

    for(int i = LOWEST_ROW - 5; i > LOWEST_ROW - track_state.longest_white_len + 5; i--){
        if(L_boundary[i] >= 10 &&
            //L_boundary[i] < TRACK_W/2 &&
            L_boundary[i] > L_boundary[i - 3] && 
            L_boundary[i] > L_boundary[i + 3] &&
            L_boundary[i - 3] > L_boundary[i - 5] &&
            L_boundary[i + 3] > L_boundary[i + 5]){
            Right_Island_Left_Inflection_Ponit = i;//存储突变点
            return 1;
           }
        }

    return 0;
}

bool Left_Island_Right_Inflection()//左岛屿右突变点判断
{
    for(int i = LOWEST_ROW - 5; i > LOWEST_ROW - track_state.longest_white_len + 5; i--){
        if(R_boundary[i] <= 150 &&
            //R_boundary[i] < TRACK_W/2 &&
            R_boundary[i] < R_boundary[i - 3] && 
            R_boundary[i] < R_boundary[i + 3] &&
            R_boundary[i - 3] <= R_boundary[i - 5] &&
            R_boundary[i + 3] <= R_boundary[i + 5]){
            Left_Island_Right_Inflection_Ponit = i;//存储突变点
            return 1;
           }
        }

    return 0;
}

bool Right_Inflection_V(int start, int end)//右突变点垂直判断
{
    if(start < end)swap(start,end);
    for(int i = start; i >= end; i--){
        if(abs((int)R_boundary[i] - (int)R_boundary[i-1]) <= 5 &&//角点的阈值可以更改
           abs((int)R_boundary[i-1] - (int)R_boundary[i-2]) <= 5 &&
           abs((int)R_boundary[i-2] - (int)R_boundary[i-3]) <= 5 &&
            ((int)R_boundary[i] - (int)R_boundary[i+1]) <= -10 &&
            ((int)R_boundary[i] - (int)R_boundary[i+1]) <= -20 ){
            Right_V_Point = i;//存储突变点
            return 1;
        }
    }
    return 0;
}

bool Left_Inflection_V(int start, int end)//左突变点垂直判断
{
    if(start < end)swap(start,end);
    for(int i = start; i >= end; i--){
        if(abs((int)L_boundary[i] - (int)L_boundary[i-1]) <= 5 &&//角点的阈值可以更改
           abs((int)L_boundary[i-1] - (int)L_boundary[i-2]) <= 5 &&
           abs((int)L_boundary[i-2] - (int)L_boundary[i-3]) <= 5 &&
            ((int)L_boundary[i] - (int)L_boundary[i+3]) >= 10 &&
            ((int)L_boundary[i] - (int)L_boundary[i+4]) >= 20 ){
            Left_V_Point = i;//存储突变点
            return 1;
        }
    }
    return 0;
}

bool Check_Right_Island()//右岛屿判断
{
    if(track_state.longest_white_len < 40)return 0;//右岛屿最短宽度要求
    for(int i = LOWEST_ROW; i >= LOWEST_ROW - 60; i--)
    {
        if((int)L_boundary[i] - 2 > (int)L_boundary[i - 1] || fabs((int)L_boundary[i] - (int)L_boundary[i - 1]) > 5)return 0;//右边界突变点判断
    }
        
    
    if(!(Right_Inflection() || Right_Inflection_V(LOWEST_ROW, 10)))return 0;//右突变点判断

    if(!L_Straight_Judge())return 0;//左边界直线判断


    if(Right_Inflection())//右突变点判断
    {    

        if(!Right_Monotonic_Mutation(Right_Inflection_Point - 5, LOWEST_ROW - track_state.longest_white_len + 4))return 0;//右突变点单调突变判断
        if(Right_Inflection_Point < Right_Mutation_Point)return 0;//突变点判断

        int single_time = 0, double_time = 0;//单跳次数，双跳次数

        for(int i = Right_Inflection_Point; i >= Right_Mutation_Point; i--){
            if(fabs((int)L_boundary[i] - (int)L_boundary[i - 1]) > 5)return 0;//突变点突变程度判断
            if(L_boundary[i] != 0 && R_boundary[i] == TRACK_W - 1)single_time++;//单跳次数
        }

        for(int i = fmin(Right_Inflection_Point + 1, LOWEST_ROW); i >= fmax(Right_Mutation_Point - 1, 0); i--){
            if(L_boundary[i] == 0)double_time++;//双跳次数
        }

        if(single_time < 5 || double_time > 0)return 0;//单跳次数判断，双跳次数判断

        IslandCtrl.state = ISLAND_STATE_ENTRY_DETECTED;//右岛屿状态
        return 1;
    }
    else if(Right_Inflection_V(LOWEST_ROW, 10))//右突变点垂直判断
    {
        if(!Right_Monotonic_Mutation(LOWEST_ROW, Right_V_Point + 10))return 0;//右突变点垂直单调突变判断
        if(Right_Mutation_Point < Right_V_Point || Right_V_Point > 30)return 0;//突变点判断
        if(abs((int)R_boundary[Right_V_Point] - (int)L_boundary[Right_V_Point]) > TRACK_W / 2)return 0;//垂直突变点与水平突变点的距离判断

        int single_time = 0, double_time = 0;//单跳次数，双跳次数

        for(int i = Right_Mutation_Point; i >= Right_V_Point; i--){
            if(fabs((int)L_boundary[i] - (int)L_boundary[i - 1]) > 5)return 0;//突变点突变程度判断
            if(L_boundary[i] != 0 && R_boundary[i] == TRACK_W - 1)single_time++;//单跳次数
        }

        for(int i = fmin(Right_Mutation_Point + 1, LOWEST_ROW); i >= fmax(Right_V_Point - 1, 0); i--){
            if(L_boundary[i] == 0)double_time++;//双跳次数
        }
        if(single_time < 1 || double_time > 0)return 0;//单跳次数判断，双跳次数判断
        //printf("II?\r\n");
        IslandCtrl.state = ISLAND_STATE_ENTRY_DETECTED;//右岛屿状态
        return 1;
    }
    return 0;
}

bool Check_Left_Island()//左岛屿判断
{
    if(track_state.longest_white_len < 40)return 0;//左岛屿最短宽度要求
    for(int i = LOWEST_ROW; i >= LOWEST_ROW - 60; i--)//左边界突变点判断
    {
        if((int)R_boundary[i] + 2 < (int)R_boundary[i - 1] || fabs((int)R_boundary[i] - (int)R_boundary[i - 1]) > 5)return 0;//左边界突变点判断
    }
        
    
    if(!(Left_Inflection() || Left_Inflection_V(LOWEST_ROW, 10)))return 0;//左突变点判断

    if(!R_Straight_Judge())return 0;//左边界直线判断

    if(Left_Inflection())//左突变点判断
    {    

        if(!Left_Monotonic_Mutation(Left_Inflection_Point - 5, LOWEST_ROW - track_state.longest_white_len + 4))return 0;//左突变点单调突变判断
        if(Left_Inflection_Point < Left_Mutation_Point)return 0;//左突变点判断

        int single_time = 0, double_time = 0;//单跳次数，双跳次数

        for(int i = Left_Inflection_Point; i >= Left_Mutation_Point; i--){
            if(fabs((int)R_boundary[i] - (int)R_boundary[i - 1]) > 5)return 0;//左突变点突变程度判断
            if(R_boundary[i] != TRACK_W - 1 && L_boundary[i] == 0)single_time++;//单跳次数
        }

        for(int i = fmin(Left_Inflection_Point + 1, LOWEST_ROW); i >= fmax(Left_Mutation_Point - 1, 0); i--){//双跳次数
            if(R_boundary[i] == TRACK_W - 1)double_time++;
        }

        if(single_time < 5 || double_time > 0)return 0;//单跳次数判断，双跳次数判断

        IslandCtrl.state = ISLAND_STATE_ENTRY_DETECTED;//左岛屿状态

        return 1;

    }
    else if(Left_Inflection_V(LOWEST_ROW, 10))
    {
        if(!Left_Monotonic_Mutation(LOWEST_ROW, Left_V_Point + 10))return 0;//左突变点垂直单调突变判断
        if(Left_Mutation_Point < Left_V_Point || Left_V_Point > 30)return 0;//左突变点判断
        if(abs((int)R_boundary[Left_V_Point] - (int)L_boundary[Left_V_Point]) > TRACK_W / 2)return 0;//垂直突变点与水平突变点的距离判断

        int single_time = 0, double_time = 0;//单跳次数，双跳次数

        for(int i = Left_Mutation_Point; i >= Left_V_Point; i--){//单跳次数
            if(fabs((int)R_boundary[i] - (int)R_boundary[i - 1]) > 5)return 0;//左突变点突变程度判断
            if(R_boundary[i] != TRACK_W - 1 && L_boundary[i] == 0)single_time++;//单跳次数
        }

        for(int i = fmin(Left_Mutation_Point + 1, LOWEST_ROW); i >= fmax(Left_V_Point - 1, 0); i--){//双跳次数
            if(R_boundary[i] == TRACK_W - 1)double_time++;//双跳次数
        }
        if(single_time < 1 || double_time > 0)return 0;//单跳次数判断，双跳次数判断
        IslandCtrl.state = ISLAND_STATE_ENTRY_DETECTED;//左岛屿状态
        return 1;
    }
    return 0;
}

void Island_Detect()//岛屿检测
{

    // printf("%d\r\n",Island_State);
    // printf("%d\r\n",Longest_White_Column);
    // printf("M:[%d,%d]\r\n",Left_Mutation_Point, L_boundary[Left_Mutation_Point]);
    // printf("I:[%d,%d]\r\n",Left_Inflection_Point, L_boundary[Left_Inflection_Point]);
    // printf("V:[%d,%d]\r\n",Left_V_Point, L_boundary[Left_V_Point]);
    if(!track_state.is_island) //岛屿状态判断
    {
        if(Check_Right_Island())//右岛屿判断
        {
            IslandCtrl.dir = RIGHT_ISLAND;//右岛屿方向
            track_state.is_island = true;//右岛屿状态
        }
        else if(Check_Left_Island())//左岛屿判断
        {
            IslandCtrl.dir = LEFT_ISLAND;//左岛屿方向
            track_state.is_island = true;//左岛屿状态
        }
    }

    if(IslandCtrl.dir == RIGHT_ISLAND){//右岛屿状态
        switch(IslandCtrl.state)
        {
            case ISLAND_STATE_ENTRY_DETECTED:{
                if(Right_Monotonic_Mutation(LOWEST_ROW, 40) != 0)//右突变点单调突变判断
                {
                    if(
                    fabs((int)R_boundary[Right_Mutation_Point + 3] - (int)R_boundary[Right_Mutation_Point - 3]) < 20 &&
                    fabs((int)R_boundary[Right_Mutation_Point + 2] - (int)R_boundary[Right_Mutation_Point - 2]) < 20 &&
                    fabs((int)R_boundary[Right_Mutation_Point + 1] - (int)R_boundary[Right_Mutation_Point - 1]) < 20)
                    {
                        IslandCtrl.state = ISLAND_STATE_APPROACHING;//右岛屿状态
                    }
                    else
                    {
                        for(int i = 0; i < TRACK_H; i++)
                        {
                            R_boundary[i] = fmin((int)L_boundary[i] + Road_width[i], TRACK_W - 1 ); //右边界计算
                        }
                    }
                }
                else
                {
                    for(int i = 0; i < TRACK_H; i++)
                    {
                        R_boundary[i] = fmin((int)L_boundary[i] + Road_width[i], TRACK_W - 1 ); //右边界计算
                           
                    }
                }
                break;
            }
            case ISLAND_STATE_APPROACHING:{
                if(Right_Inflection_V(LOWEST_ROW - 20, 10))//右突变点垂直突变判断
                {
                    
                    if(Right_V_Point > Island_Time)//右突变点判断
                    {
                        IslandCtrl.state = ISLAND_STATE_ENTERING;//右岛屿状态
                    }
                    else
                    {
                        for(int i = 0; i < TRACK_H; i++)
                        {
                            R_boundary[i] = fmin((int)L_boundary[i] + Road_width[i], TRACK_W - 1 ); //右边界计算
                        }
                    }

                }
                else
                {
                    for(int i = 0; i < TRACK_H; i++)
                    {
                        R_boundary[i] = fmin((int)L_boundary[i] + Road_width[i], TRACK_W - 1 ); //右边界计算
                    }
                }
                break;
            }
            case ISLAND_STATE_ENTERING:{
                if(Right_Inflection_V(LOWEST_ROW - 20, 10))//右突变点垂直突变判断
                {
                    float slope, intercept;
                    //auto result = Right_Point2Point(Left_V_Point, L_boundary[Left_V_Point], 75, R_boundary[75]);
                    slope = -2;
                    intercept = (float)R_boundary[Right_V_Point] - Right_V_Point * slope; //斜率，截距计算
                    for(int i = LOWEST_ROW; i >= 0; i--)
                    {
                        int x = cvRound(slope * i + intercept); //x计算
                        L_boundary[i] = fmin(fmax(x, 0), TRACK_W - 1); //左边界计算
                    }
                    track_state.longest_white_len = track_state.white_pixel_count[R_boundary[Right_V_Point]];//最长白线计算
                    for(int i = R_boundary[Right_V_Point]; i <= TRACK_W - 1; i++)
                    {
                        if(track_state.white_pixel_count[i] > track_state.longest_white_len)//最长白线判断
                        {
                            track_state.longest_white_len = track_state.white_pixel_count[i];//最长白线计算
                            track_state.longest_white_x = i;//最长白线x计算
                        }
                    }
                    for(int i = 0; i < TRACK_H; i++)
                    {
                        R_boundary[i] = TRACK_W - 1;//右边界计算
                    }

                    for(int i = LOWEST_ROW; i >= LOWEST_ROW - track_state.longest_white_len; i--){
                        for(int j = track_state.longest_white_x; j < TRACK_W - 1; j++){
                            if(img_bin[i][j] == 255 && img_bin[i][j + 1] == 0 ){
                                R_boundary[i] = j + 1;//右边界计算
                                break;
                            }
                        }
                    }

                    check1 = 1; //标志位

                    if(Right_V_Point > 30)//右突变点判断
                    {
                        IslandCtrl.state = ISLAND_STATE_INSIDE;//右岛屿状态
                    }
                }
                else
                {
                    if(check1)//标志位
                    {
                        memset(R_boundary, TRACK_W - 1, TRACK_H); //右边界计算
                        for(int i = 0; i < TRACK_H; i++)
                        {
                            L_boundary[i] = fmax((int)R_boundary[i] - curve_width[i], 0 );        //左边界计算
                        }
                    }
                    else
                    {
                        for(int i = 0; i < TRACK_H; i++)
                        {
                            R_boundary[i] = fmin((int)L_boundary[i] + Road_width[i], TRACK_W - 1 );      //右边界计算
                          
                        }
                    }
                }
                break;
            }
            case ISLAND_STATE_INSIDE:{
                
                if(Right_Inflection_V(LOWEST_ROW - 20, 10))//右突变点垂直突变判断
                {
                    track_state.longest_white_len = track_state.white_pixel_count[R_boundary[Right_V_Point]];//最长白线计算
                    for(int i = R_boundary[Right_V_Point]; i <= TRACK_W - 1; i++)//最长白线判断
                    {
                        if(track_state.white_pixel_count[i] > track_state.longest_white_len)//最长白线判断
                        {
                            track_state.longest_white_len = track_state.white_pixel_count[i];//最长白线计算
                            track_state.longest_white_x = i;//最长白线x计算
                        }
                    }
                    memset(R_boundary, TRACK_W - 1, TRACK_H); //右边界计算
                    for(int i = LOWEST_ROW; i >= LOWEST_ROW - track_state.longest_white_len; i--){ //右边界计算
                        for(int j = track_state.longest_white_x; j > 0; j--){ //左边界计算
                            if(img_bin[i][j] == 255 && img_bin[i][j - 1] == 0 ){ //左边界计算
                                L_boundary[i] = j - 1;//左边界计算
                                break;
                            }
                        }
                        for(int j = track_state.longest_white_x; j < TRACK_W - 1; j++){ //右边界计算
                            if(img_bin[i][j] == 255 && img_bin[i][j + 1] == 0 ){ //右边界计算
                                R_boundary[i] = j + 1;//右边界计算
                                break;
                            }
                        }
                    }
                    

                    if(Left_Inflection_V(LOWEST_ROW, 10))//左突变点垂直突变判断
                    {
                        Left_Point2Point(Left_V_Point, L_boundary[Left_V_Point], 75, L_boundary[75]);//左突变点2点拟合
                    }
                    else
                    {
                        memset(R_boundary, TRACK_W - 1, TRACK_H); //右边界计算
                        for(int i = 0; i < TRACK_H; i++) //左边界计算
                        {
                            L_boundary[i] = fmax((int)R_boundary[i] - curve_width[i], 0 );        //左边界计算
                        }
                    }

                }
                else
                {
                    IslandCtrl.state = ISLAND_STATE_EXIT_DETECTED;//右突变点水平突变判断
                }

                break;
            }
            case ISLAND_STATE_EXIT_DETECTED:{
                if(Right_Island_Left_Inflection())//右岛屿左突变点判断
                {
                    IslandCtrl.state = ISLAND_STATE_EXIT_APPROACHING;//右岛屿左突变点2点拟合
                }
                break;
            }
            case ISLAND_STATE_EXIT_APPROACHING:{
                if(Right_Island_Left_Inflection())//右突变点水平突变判断
                {
                    Left_Point2Point(Right_Island_Left_Inflection_Ponit, L_boundary[Right_Island_Left_Inflection_Ponit],
                                    20, 150);//左突变点2点拟合
                }
                else
                {
                    IslandCtrl.state = ISLAND_STATE_EXITING;//右岛屿左突变点水平突变判断
                }
                break;
            }
            case ISLAND_STATE_EXITING:{
                Left_Point2Point(70, L_boundary[70],
                                    LOWEST_ROW - track_state.longest_white_len, 150);//左突变点2点拟合
                
                if(//track_state.longest_white_x > TRACK_W / 2 &&
                //track_state.longest_white_len - track_state.white_pixel_count[TRACK_W / 2] > 3
                    Find_EXIT_V_Point(LOWEST_ROW - 20, 10))//寻找出环时的V点
                {
                    IslandCtrl.state = ISLAND_STATE_FINDEXIT_V;//右岛屿左突变点水平突变判断
                }

                break;
            }
            case ISLAND_STATE_FINDEXIT_V:{
                if(Find_EXIT_V_Point(LOWEST_ROW - 20, 10)){
                    Right_Point2Point(Right_exit_V_Point, R_boundary[Right_exit_V_Point], TRACK_W - 5, TRACK_H - 5);
                }else{
                    IslandCtrl.state = ISLAND_STATE_ADJUSTING;
                }
                break;
            }
                
            case ISLAND_STATE_ADJUSTING:{
                if(Check_Lost_R())//右突变点水平突变判断
                {
                    for(int i = 0; i < TRACK_H; i++)
                    {
                        R_boundary[i] = fmin((int)L_boundary[i] + Road_width[i], TRACK_W - 1);        //右边界计算
                    }
                }
                else
                {
                    IslandCtrl.state = ISLAND_STATE_FINISHED;//右突变点水平突变判断
                }
                break;
            }
            case ISLAND_STATE_FINISHED:{
                IslandCtrl.dir = NONE_ISLAND;//岛屿方向
                IslandCtrl.state = ISLAND_STATE_NONE;//岛屿状态
                track_state.is_island = false;//岛屿标志状态
                Right_Inflection_Point = 0;//右突变点
                Right_Mutation_Point = 0;//右突变点
                Right_V_Point = 0;//右突变点
                check1 = 0;//右突变点水平突变判断
                break;
            }
            default:
                break;
        }
    }

    if(IslandCtrl.dir == LEFT_ISLAND){//岛屿方向
        switch(IslandCtrl.state) //岛屿状态
        {
            case ISLAND_STATE_ENTRY_DETECTED:{
                if(Left_Monotonic_Mutation(LOWEST_ROW, 40) != 0)//左突变点水平突变判断
                {
                    if(
                    fabs((int)L_boundary[Left_Mutation_Point + 3] - (int)L_boundary[Left_Mutation_Point - 3]) < 20 &&
                    fabs((int)L_boundary[Left_Mutation_Point + 2] - (int)L_boundary[Left_Mutation_Point - 2]) < 20 &&
                    fabs((int)L_boundary[Left_Mutation_Point + 1] - (int)L_boundary[Left_Mutation_Point - 1]) < 20)
                    {
                        IslandCtrl.state = ISLAND_STATE_APPROACHING;//左突变点水平突变判断
                    }
                    else
                    {
                        for(int i = 0; i < TRACK_H; i++)
                        {
                            L_boundary[i] = fmax((int)R_boundary[i] - Road_width[i], 0 );  //左边界计算
                        }
                    }
                }
                else
                {
                    for(int i = 0; i < TRACK_H; i++)
                    {
                        L_boundary[i] = fmax((int)R_boundary[i] - Road_width[i], 0 );        //左边界计算
                    }
                }
                break;
            }
            case ISLAND_STATE_APPROACHING:{
                if(Left_Inflection_V(LOWEST_ROW - 20, 10))//左突变点垂直突变判断
                {
                    
                    if(Left_V_Point > Island_Time)//左突变点垂直突变判断
                    {
                        IslandCtrl.state = ISLAND_STATE_ENTERING;//左突变点垂直突变判断
                    }
                    else
                    {
                        for(int i = 0; i < TRACK_H; i++)
                        {
                            L_boundary[i] = fmax((int)R_boundary[i] - Road_width[i], 0 );        //左边界计算
                        }
                    }

                }
                else
                {
                    for(int i = 0; i < TRACK_H; i++)
                    {
                        L_boundary[i] = fmax((int)R_boundary[i] - Road_width[i], 0 );        //左边界计算
                    }
                }
                break;
            }
            case ISLAND_STATE_ENTERING:{
                if(Left_Inflection_V(LOWEST_ROW - 20, 10))//左突变点垂直突变判断
                {
                    float slope, intercept; //斜率和截距
                    //auto result = Right_Point2Point(Left_V_Point, L_boundary[Left_V_Point], 75, R_boundary[75]);
                    slope = 2; //计算斜率
                    intercept = (float)L_boundary[Left_V_Point] - Left_V_Point * slope; //计算截距
                    for(int i = LOWEST_ROW; i >= 0; i--)
                    {
                        int x = cvRound(slope * i + intercept); //计算右边界
                        R_boundary[i] = fmin(fmax(x, 0), TRACK_W - 1); //计算右边界
                    }
                    track_state.longest_white_len = track_state.white_pixel_count[L_boundary[Left_V_Point]]; //计算最长白列
                    for(int i = L_boundary[Left_V_Point]; i >= 0; i--)
                    {
                        if(track_state.white_pixel_count[i] > track_state.longest_white_len)
                        {
                            track_state.longest_white_len = track_state.white_pixel_count[i]; //计算最长白列
                            track_state.longest_white_x = i; //计算最长白列的x坐标
                        }
                    }
                    for(int i = 0; i < TRACK_H; i++)
                    {
                        L_boundary[i] = 0; //左边界计算
                    }

                    for(int i = LOWEST_ROW; i >= LOWEST_ROW - track_state.longest_white_len; i--){
                        for(int j = track_state.longest_white_x; j > 0; j--){
                            if(img_bin[i][j] == 255 && img_bin[i][j - 1] == 0 ){
                                L_boundary[i] = j - 1; //左边界计算
                                break;
                            }
                        }
                    }

                    check1 = 1; //标记

                    if(Left_V_Point > 30)
                    {
                        IslandCtrl.state = ISLAND_STATE_INSIDE; //左突变点垂直突变判断
                    }
                }
                else
                {
                    if(check1)
                    {
                        memset(L_boundary, 0, TRACK_H); //左边界计算
                        for(int i = 0; i < TRACK_H; i++)
                        {
                            R_boundary[i] = fmin((int)L_boundary[i] + curve_width[i], TRACK_W - 1 );      //右边界计算
                          
                        }
                    }
                    else
                    {
                        for(int i = 0; i < TRACK_H; i++)
                        {
                            L_boundary[i] = fmax((int)R_boundary[i] - Road_width[i], 0 );        //左边界计算
                        }
                    }
                    
                }
                break;
            }
            case ISLAND_STATE_INSIDE:{
                
                if(Left_Inflection_V(LOWEST_ROW - 20, 10))
                {
                    track_state.longest_white_len = track_state.white_pixel_count[L_boundary[Left_V_Point]]; //计算最长白列
                    for(int i = L_boundary[Left_V_Point]; i >= 0; i--)
                    {
                        if(track_state.white_pixel_count[i] > track_state.longest_white_len)
                        {
                            track_state.longest_white_len = track_state.white_pixel_count[i]; //计算最长白列
                            track_state.longest_white_x = i; //计算最长白列的x坐标
                        }
                    }
                    memset(L_boundary, 0, TRACK_H);
                    for(int i = LOWEST_ROW; i >= LOWEST_ROW - track_state.longest_white_len; i--){
                        for(int j = track_state.longest_white_x; j > 0; j--){
                            if(img_bin[i][j] == 255 && img_bin[i][j - 1] == 0 ){
                                L_boundary[i] = j - 1; //左边界计算
                                break;
                            }
                        }
                        for(int j = track_state.longest_white_x; j < TRACK_W - 1; j++){
                            if(img_bin[i][j] == 255 && img_bin[i][j + 1] == 0 ){
                                R_boundary[i] = j + 1; //右边界计算
                                break;
                            }
                        }
                    }

                    if(Right_Inflection_V(LOWEST_ROW - 20, 10))
                    {
                        Right_Point2Point(Right_V_Point, R_boundary[Right_V_Point], 75, R_boundary[75]); //计算右边界
                    }
                    else
                    {
                        memset(L_boundary, 0, TRACK_H); //左边界计算
                        for(int i = 0; i < TRACK_H; i++)
                        {
                            R_boundary[i] = fmin((int)L_boundary[i] + curve_width[i], TRACK_W - 1 );     //右边界计算
                          
                           
                        }
                    }

                }
                else
                {
                    IslandCtrl.state = ISLAND_STATE_EXIT_DETECTED; //左突变点垂直突变判断
                }

                break;
            }
            case ISLAND_STATE_EXIT_DETECTED:{
                if(Left_Island_Right_Inflection())
                {
                    IslandCtrl.state = ISLAND_STATE_EXIT_APPROACHING; //左突变点垂直突变判断
                }
                break;
            }
            case ISLAND_STATE_EXIT_APPROACHING:{
                if(Left_Island_Right_Inflection())
                {
                    Right_Point2Point(Left_Island_Right_Inflection_Ponit, R_boundary[Left_Island_Right_Inflection_Ponit],
                                    20, 10);//计算右边界
                }
                else
                {
                    IslandCtrl.state = ISLAND_STATE_EXITING; //左突变点垂直突变判断
                }
                break;
            }
            case ISLAND_STATE_EXITING:{
                Right_Point2Point(70, R_boundary[70],
                                    LOWEST_ROW - track_state.longest_white_len, 10); //计算右边界
                
                if(track_state.longest_white_x < TRACK_W / 2 &&
                track_state.longest_white_len - track_state.white_pixel_count[TRACK_W / 2] > 3) //左突变点垂直突变判断
                {
                    IslandCtrl.state = ISLAND_STATE_ADJUSTING; //左突变点垂直突变判断
                }

                break;
            }
            case ISLAND_STATE_ADJUSTING:{
                if(Check_Lost_L()) //左突变点垂直突变判断
                {
                    for(int i = 0; i < TRACK_H; i++)
                    {
                        L_boundary[i] = fmax((int)R_boundary[i] - Road_width[i], 0);    //左边界计算
                        
                    }
                }
                else
                {
                    IslandCtrl.state = ISLAND_STATE_FINISHED; //左突变点垂直突变判断
                }
                break;
            }
            case ISLAND_STATE_FINISHED:{
                IslandCtrl.dir = NONE_ISLAND;//岛屿方向
                IslandCtrl.state = ISLAND_STATE_NONE;//岛屿状态
                track_state.is_island = false;//岛屿标志状态
                Left_Inflection_Point = 0;//左突变点
                Left_Mutation_Point = 0;//左突变点
                Left_V_Point = 0;//左突变点
                check1 = 0;//左突变点水平突变判断
                break;
            }
            default:
                break;
        }
    }
    //printf("%d",IslandCtrl.state);
}

bool Check_Lost_L()//检查左突变点是否丢失
{
    for(int i = LOWEST_ROW - 10; i > LOWEST_ROW - 15 ; i--)
    {
        if(L_boundary[i] == 0)return 1;//左突变点丢失
    }
    return 0;
}

bool Check_Lost_R()//检查右突变点是否丢失
{
    for(int i = LOWEST_ROW - 10; i > LOWEST_ROW - 15 ; i--)
    {
        if(R_boundary[i] == TRACK_W - 1)return 1;//右突变点丢失
    }
    return 0;
}

bool Check_Straight_L()//检查左突变点是否为直线
{
        float k = (float)(L_boundary[LOWEST_ROW] - L_boundary[15]) / (float)(LOWEST_ROW - 15);
        float sumErr = 0;
        for(int i = LOWEST_ROW; i > 15 ; i--)
        {
            if(L_boundary[i] == 0)return 0;
            float predict = L_boundary[LOWEST_ROW] + k * (i - LOWEST_ROW);
            float error = predict - L_boundary[i];
            sumErr += error * error;
        }
        sumErr /= (float)(LOWEST_ROW - 15);
        if(sumErr < 1)return 1;//左突变点为直线
        return 0; 
}

bool Check_Straight_R()//检查右突变点是否为直线
{
        float k = (float)(R_boundary[LOWEST_ROW] - R_boundary[15]) / (float)(LOWEST_ROW - 15);
        float sumErr = 0;
        for(int i = LOWEST_ROW; i > 15 ; i--)
        {
            if(L_boundary[i] == TRACK_W - 1)return 0;//右突变点为直线
            float predict = R_boundary[LOWEST_ROW] + k * (i - LOWEST_ROW);
            float error = predict - R_boundary[i];
            sumErr += error * error;
        }
        sumErr /= (float)(LOWEST_ROW - 15);
        if(sumErr < 1)return 1;//右突变点为直线
        return 0; 
}

bool Check_White_L()//检查左突变点是否为白色
{
    for(int i = LOWEST_ROW; i > 15 ; i--)
    {
        if(img_bin[i][1] == 255)return 1;//左突变点为白色
    }
    return 0;
}

bool Check_White_R()//检查右突变点是否为白色
{
    for(int i = LOWEST_ROW; i > 15 ; i--)
    {
        if(img_bin[i][TRACK_W - 2] == 255)return 1;//右突变点为白色
    }
    return 0;
}