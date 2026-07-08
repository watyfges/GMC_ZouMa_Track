#include "cross.h"//十字路口检测

using namespace std;


extern uint8 L_boundary[TRACK_H], M_boundary[TRACK_H], R_boundary[TRACK_H];//左右边界

int Left_Down_Find,Right_Down_Find,Left_Up_Find,Right_Up_Find;//十字路口角点坐标
float slope, intercept;//下边界线的斜率和截距

void Find_Down_Point(int start,int end){//查找下边界的角点

    Left_Down_Find = 0;//初始化左下角点坐标
    Right_Down_Find = 0;//初始化右下角点坐标

    if(start < end){//确保start大于end
        swap(start,end);
    }
    if(start > LOWEST_ROW - 5){//确保start不小于最低行-5
        start = LOWEST_ROW - 5;//确保start不小于最低行-5
    }
    // if(end < LOWEST_ROW - 5 - track_state.longest_white_len){//确保end不小于最低行-5-最长白色列数
    //     end = LOWEST_ROW - 5 - track_state.longest_white_len;
    //     end = fmax(5,end);
    // }

    for(int i = start; i >= end; i--){
        if(Left_Down_Find == 0 &&//只找第一个符合条件的点
           abs((int)L_boundary[i] - (int)L_boundary[i+1]) <= 5 &&//角点的阈值可以更改
           abs((int)L_boundary[i+1] - (int)L_boundary[i+2]) <= 5 &&
           abs((int)L_boundary[i+2] - (int)L_boundary[i+3]) <= 5 &&
              ((int)L_boundary[i] - (int)L_boundary[i-2]) >= 8 &&
              ((int)L_boundary[i] - (int)L_boundary[i-3]) >= 15 &&
              ((int)L_boundary[i] - (int)L_boundary[i-4]) >= 15){
            Left_Down_Find = i;//获取行数即可
        }
        if(Right_Down_Find == 0 &&//只找第一个符合条件的点
           abs((int)R_boundary[i] - (int)R_boundary[i+1]) <= 5 &&//角点的阈值可以更改
           abs((int)R_boundary[i+1] - (int)R_boundary[i+2]) <= 5 &&
           abs((int)R_boundary[i+2] - (int)R_boundary[i+3]) <= 5 &&
              ((int)R_boundary[i] - (int)R_boundary[i-2]) <= -8 &&
              ((int)R_boundary[i] - (int)R_boundary[i-3]) <= -15 &&
              ((int)R_boundary[i] - (int)R_boundary[i-4]) <= -15){
            Right_Down_Find = i;
        }
        if(Left_Down_Find!=0&&Right_Down_Find!=0){//两个找到就退出
                    break;
        }
    }
}

void Find_Up_Point(int start,int end){//查找上边界的角点

    Left_Up_Find = 0;//初始化左上角点坐标
    Right_Up_Find = 0;//初始化右上角点坐标

    if(start < end){//确保start大于end
        swap(start,end);
    }
    if(start > LOWEST_ROW - 5){//确保start不小于最低行-5
        start = LOWEST_ROW - 5;
    }
    if(end < LOWEST_ROW - 5 - track_state.longest_white_len){//确保end不小于最低行-5-最长白色列数
        end = LOWEST_ROW - 5 - track_state.longest_white_len;
        end = fmax(5,end);
    }

    for(int i = start; i >= end; i--){
        if(Left_Up_Find == 0 &&//只找第一个符合条件的点
           abs((int)L_boundary[i] - (int)L_boundary[i-1]) <= 5 &&//角点的阈值可以更改
           abs((int)L_boundary[i-1] - (int)L_boundary[i-2]) <= 5 &&
           abs((int)L_boundary[i-2] - (int)L_boundary[i-3]) <= 5 &&
              ((int)L_boundary[i] - (int)L_boundary[i+2]) >= 8 &&
              ((int)L_boundary[i] - (int)L_boundary[i+3]) >= 15 &&
              ((int)L_boundary[i] - (int)L_boundary[i+4]) >= 15){
            Left_Up_Find = i;//获取行数即可
        }
        if(Right_Up_Find == 0 &&//只找第一个符合条件的点
           abs((int)R_boundary[i] - (int)R_boundary[i-1]) <= 5 &&//角点的阈值可以更改
           abs((int)R_boundary[i-1] - (int)R_boundary[i-2]) <= 5 &&
           abs((int)R_boundary[i-2] - (int)R_boundary[i-3]) <= 5 &&
              ((int)R_boundary[i] - (int)R_boundary[i+2]) <= -8 &&
              ((int)R_boundary[i] - (int)R_boundary[i+3]) <= -15 &&
              ((int)R_boundary[i] - (int)R_boundary[i+4]) <= -15){
            Right_Up_Find = i;
        }
        if(Left_Up_Find !=0 && Right_Up_Find != 0){//两个找到就退出
            break;
        }
    }
}

// 最小二乘法计算斜率
float calculateSlope(int begin, int end, uint8 border[TRACK_H]) {//计算斜率
    float sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;//初始化累加变量
    static float lastValidSlope = 0; // 保留上次有效斜率

    for (int y = begin; y <= end; ++y) {//遍历边界上的点
        sumX += y;//累加x坐标
        sumY += border[y];//累加y坐标
        sumXY += y * border[y];//累加x*y坐标
        sumXX += y * y;//累加x*x坐标
    }

    int numPoints = end - begin + 1;//计算点的数量
    float denominator = numPoints * sumXX - sumX * sumX;//计算分母

    // 防止除以零
    if (fabs(denominator) < 1e-5) return lastValidSlope;//防止除以零

    float slope = (numPoints * sumXY - sumX * sumY) / denominator;//计算斜率
    lastValidSlope = slope; // 更新保留值
    return slope;//返回斜率
}

// 最小二乘法计算斜率连线
float calculateSlopeConnect(int first_y, int second_y, uint8 border[TRACK_H]) {//计算斜率连线
    float sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;//初始化累加变量
    static float lastValidSlope = 0; // 保留上次有效斜率

    sumX = first_y + second_y;//累加x坐标
    sumY = border[first_y] + border[second_y];//累加y坐标
    sumXY = first_y * border[first_y] + second_y * border[second_y];//累加x*y坐标
    sumXX = first_y * first_y + second_y * second_y;//累加x*x坐标

    int numPoints = 2;//计算点的数量
    float denominator = numPoints * sumXX - sumX * sumX;//计算分母

    // 防止除以零
    if (fabs(denominator) < 1e-5) return lastValidSlope;//防止除以零

    float slope = (numPoints * sumXY - sumX * sumY) / denominator;//计算斜率
    lastValidSlope = slope; // 更新保留值
    return slope;//返回斜率
}

// 计算斜率和截距
void calculateSlopeIntercept(int startY, int endY, 
                            uint8 border[TRACK_H],
                            float& slope, 
                            float& intercept) {//计算斜率和截距

    int sumX = 0, sumY = 0; // 初始化累加变量
    startY = fmax(startY, 0); // 确保起始行不小于0

    // 统计区间数据
    for (int y = startY; y <= endY; ++y) { // 遍历区间内的点
        sumX += y; // 累加x坐标
        sumY += border[y]; // 累加y坐标
    }

    int numPoints = endY - startY + 1; // 计算点的数量
    if (numPoints == 0) { // 如果没有点，则斜率和截距都为0
        slope = 0;
        intercept = 0;
        return;
    }

    // 计算均值
    float meanX = static_cast<float>(sumX) / numPoints; // 计算x坐标的均值
    float meanY = static_cast<float>(sumY) / numPoints; // 计算y坐标的均值

    // 获取斜率
    slope = calculateSlope(startY, endY, border); // 获取斜率
    intercept = meanY - slope * meanX; // 计算截距
}

void Lengthen_Left_Boundry(int start){//延长左边界

    calculateSlopeIntercept(start - 4, start, L_boundary, slope, intercept);//计算左边界的斜率和截距
    for (int y = start + 1; y < TRACK_H; ++y) {//遍历左边界上的点
            int x = cvRound(slope * y + intercept);//计算x坐标
            L_boundary[y] = fmin(fmax(x, 0), TRACK_W - 1);//将x坐标限制在图像宽度内
    }
}

void Lengthen_Right_Boundry(int start){//延长右边界

    calculateSlopeIntercept(start - 4, start, R_boundary, slope, intercept);//计算右边界的斜率和截距
    for (int y = start + 1; y < TRACK_H; ++y) {//遍历右边界上的点
            int x = cvRound(slope * y + intercept);//计算x坐标
            R_boundary[y] = fmin(fmax(x, 0), TRACK_W - 1);//将x坐标限制在图像宽度内
    }
}

void Left_Add_Line (int Up,int Down){//在左边界上添加一条线

    //统计区间数据
    int sumX = Up + Down, sumY = L_boundary[Up] + L_boundary[Down];
    // 计算均值
    float meanX = static_cast<float>(sumX) / 2;
    float meanY = static_cast<float>(sumY) / 2;
    // 获取斜率
    slope = calculateSlopeConnect(Up, Down, L_boundary);//计算左边界上的线的斜率
    intercept = meanY - slope * meanX;//计算左边界上的线的截距

    for (int y = Up + 1; y < Down; ++y) {//遍历左边界上的线的点
            int x = cvRound(slope * y + intercept);//计算x坐标
            L_boundary[y] = fmin(fmax(x, 0), TRACK_W - 1);//将x坐标限制在图像宽度内
    }
}

void Right_Add_Line(int Up,int Down){//在右边界上添加一条线
    
    //统计区间数据
    int sumX = Up + Down, sumY = R_boundary[Up] + R_boundary[Down];
    // 计算均值
    float meanX = static_cast<float>(sumX) / 2;
    float meanY = static_cast<float>(sumY) / 2;
    // 获取斜率
    slope = calculateSlopeConnect(Up, Down, R_boundary);//计算右边界上的线的斜率
    intercept = meanY - slope * meanX;//计算右边界上的线的截距

    for (int y = Up + 1; y < Down; ++y) {//遍历右边界上的线的点
            int x = cvRound(slope * y + intercept);//计算x坐标
            R_boundary[y] = fmin(fmax(x, 0), TRACK_W - 1);//将x坐标限制在图像宽度内
    }
}

void Cross_Detect()
{
    int down_search_start = 0;//下角点搜索开始行
    //if(track_state.is_island && track_state.is_ramp)//与环岛互斥开
    //{

        Left_Up_Find = 0;//左上角点标志位
        Right_Up_Find = 0;//右上角点标志位
        
        int Both_Lost_Time = 0;

        for(int i = LOWEST_ROW; i >= LOWEST_ROW - track_state.longest_white_len; i--){//从最下面一行开始向上搜索
            if(L_boundary[i] == 0 && R_boundary[i] == TRACK_W - 1 )Both_Lost_Time++;//如果左边界在最左边，右边界在最右边，双边丢线次数加1
        }
        if(Both_Lost_Time>=10)//十字必定有双边丢线，在有双边丢线的情况下再开始找角点
        {
            Find_Up_Point(TRACK_H-1, 0);//从最下面一行开始向上搜索
            if( Left_Up_Find==0 && Right_Up_Find==0)//只要没有同时找到两个上点，直接结束
            {
                return;
            }
        }

        if(Left_Up_Find!=0 && Right_Up_Find!=0)//找到两个上点，就认为找到十字了
        {
            //printf("L_U:[%d, %d]\r\n",L_boundary[Left_Up_Find],Left_Up_Find);
            //printf("R_U:[%d, %d]\r\n",R_boundary[Right_Up_Find],Right_Up_Find);
            track_state.is_cross = true;//确定对应标志位，便于各元素互斥掉
            down_search_start = Left_Up_Find > Right_Up_Find ? Left_Up_Find : Right_Up_Find;//用两个上拐点坐标靠下者作为下点的搜索上限
            Find_Down_Point(TRACK_H-6, down_search_start+2);//在上拐点下2行作为下角点的截止行

            if(Left_Down_Find <= Left_Up_Find)
            {
                Left_Down_Find = 0;//下点不可能比上点还靠上
            }
            if(Right_Down_Find <= Right_Up_Find)
            {
                Right_Down_Find = 0;//下点不可能比上点还靠上
            }

            if(Left_Down_Find!=0&&Right_Down_Find!=0)
            {//四个点都在，无脑连线，这种情况显然很少
                Left_Add_Line (Left_Up_Find, Left_Down_Find);//左边界添加线
                Right_Add_Line(Right_Up_Find, Right_Down_Find);//右边界添加线
            }
            else if(Left_Down_Find==0 && Right_Down_Find!=0)//11//这里使用的是斜率补线
            {//三个点                                       //01
                Lengthen_Left_Boundry(Left_Up_Find);//左边界延长
                Right_Add_Line(Right_Up_Find, Right_Down_Find);//右边界添加线
            }
            else if(Left_Down_Find!=0 && Right_Down_Find==0)//11
            {//三个点                                        //10
                Left_Add_Line (Left_Up_Find, Left_Down_Find);//左边界添加线
                Lengthen_Right_Boundry(Right_Up_Find);  //右边界延长
            }
            else if(Left_Down_Find==0 && Right_Down_Find==0)//11
            {//就俩上点                                      //00
                Lengthen_Left_Boundry (Left_Up_Find);//左边界延长
                Lengthen_Right_Boundry(Right_Up_Find);//右边界延长
            }
            //printf("L_D:[%d, %d]\r\n",L_boundary[Left_Down_Find],Left_Down_Find);
            //printf("R_D:[%d, %d]\r\n",R_boundary[Right_Down_Find],Right_Down_Find);
        }
        else
        {
            track_state.is_cross = false;//没找到两个上点，就认为没找到十字
        }
    //}
}