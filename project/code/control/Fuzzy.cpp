#include "Fuzzy.h"//过于麻烦了就不使用模糊控制了，使用二次PID代替，效果差不多，二次PID好调很多
#include <algorithm>
#include <cstdint>

using namespace std;

extern float Speed;
extern float absolute_error;

float kp_m, kd_m;  // 全局变量定义

// 输入特征点定义
// #if Speed == 400
//     float EFF[7] = {-210f, -170f, -70f, 0.000000f, 70f, 170f, 210f};
//     float DFF[7] = {-30f, -20f, -10f, 0.000000f, 10f, 20f, 30f};
//     float UFF[7] =  { 0.1, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1};
// #else
//     float EFF[7] = {-51.716124f, -21.882200f, -15.165950f, 0.0f, 15.165950f, 21.882200f, 51.716124f};
//     float DFF[7] = {-0.897037f, -0.298043f, -0.093209f, 0.0f, 0.093209f, 0.298043f, 0.897037f};
//     float UFF[7] =  { 10.2, 11.5, 11.8, 12.4, 12.8, 13.2, 13.5};
// #endif

float Fuzzy_P(float E, float EC)
{ 
    float EFF[7] = {-22.0f, -16.0f, -11.0f, 0.000000f, 11.0f, 16.0f, 22.0f};
    float DFF[7] = {-1.5f, -1.0f, -0.5f, 0.000000f, 0.5f, 1.0f, 1.5f};
    float UFF[7] =  { 0.4, 0.6, 0.7, 0.9, 1.0, 1.2, 1.3};
    int rule[7][7] = {
        {6, 5, 4, 3, 2, 1, 0},                          
        {5, 4, 3, 2, 1, 0, 1},
        {4, 3, 2, 1, 0, 1, 2},
        {3, 2, 1, 0, 1, 2, 3},
        {2, 1, 0, 1, 2, 3, 4},
        {1, 0, 1, 2, 3, 4, 5},
        {0, 1, 2, 3, 4, 5, 6},
    };
    float U = 0;                                 
    float PF[2] = {0}, DF[2] = {0}, UF[4] = {0}; 

    int Pn = 0, Dn = 0, Un[4] = {0};
    float t1 = 0, t2 = 0, t3 = 0, t4 = 0, temp1 = 0, temp2 = 0;



    if (E > EFF[0] && E < EFF[6])
    {
        if (E <= EFF[1])
        {
            Pn = -2;
            PF[0] = (EFF[1] - E) / (EFF[1] - EFF[0]);
        }
        else if (E <= EFF[2])
        {
            Pn = -1;
            PF[0] = (EFF[2] - E) / (EFF[2] - EFF[1]);
        }
        else if (E <= EFF[3])
        {
            Pn = 0;
            PF[0] = (EFF[3] - E) / (EFF[3] - EFF[2]);
        }
        else if (E <= EFF[4])
        {
            Pn = 1;
            PF[0] = (EFF[4] - E) / (EFF[4] - EFF[3]);
        }
        else if (E <= EFF[5])
        {
            Pn = 2;
            PF[0] = (EFF[5] - E) / (EFF[5] - EFF[4]);
        }
        else if (E <= EFF[6])
        {
            Pn = 3;
            PF[0] = (EFF[6] - E) / (EFF[6] - EFF[5]);
        }
    }

    else if (E <= EFF[0])
    {
        Pn = -2;
        PF[0] = 1;
    }
    else if (E >= EFF[6])
    {
        Pn = 3;
        PF[0] = 0;
    }

    PF[1] = 1 - PF[0];

    if (EC > DFF[0] && EC < DFF[6])
    {
        if (EC <= DFF[1])
        {
            Dn = -2;
            DF[0] = (DFF[1] - EC) / (DFF[1] - DFF[0]);
        }
        else if (EC <= DFF[2])
        {
            Dn = -1;
            DF[0] = (DFF[2] - EC) / (DFF[2] - DFF[1]);
        }
        else if (EC <= DFF[3])
        {
            Dn = 0;
            DF[0] = (DFF[3] - EC) / (DFF[3] - DFF[2]);
        }
        else if (EC <= DFF[4])
        {
            Dn = 1;
            DF[0] = (DFF[4] - EC) / (DFF[4] - DFF[3]);
        }
        else if (EC <= DFF[5])
        {
            Dn = 2;
            DF[0] = (DFF[5] - EC) / (DFF[5] - DFF[4]);
        }
        else if (EC <= DFF[6])
        {
            Dn = 3;
            DF[0] = (DFF[6] - EC) / (DFF[6] - DFF[5]);
        }
    }
    else if (EC <= DFF[0])
    {
        Dn = -2;
        DF[0] = 1;
    }
    else if (EC >= DFF[6])
    {
        Dn = 3;
        DF[0] = 0;
    }

    DF[1] = 1 - DF[0];


    Un[0] = rule[Pn + 2][Dn + 2];
    Un[1] = rule[Pn + 3][Dn + 2];
    Un[2] = rule[Pn + 2][Dn + 3];
    Un[3] = rule[Pn + 3][Dn + 3];

    if (PF[0] <= DF[0]) 
        UF[0] = PF[0];
    else
        UF[0] = DF[0];
    if (PF[1] <= DF[0])
        UF[1] = PF[1];
    else
        UF[1] = DF[0];
    if (PF[0] <= DF[1])
        UF[2] = PF[0];
    else
        UF[2] = DF[1];
    if (PF[1] <= DF[1])
        UF[3] = PF[1];
    else
        UF[3] = DF[1];

    if (Un[0] == Un[1])
    {
        if (UF[0] > UF[1])
            UF[1] = 0;
        else
            UF[0] = 0;
    }
    if (Un[0] == Un[2])
    {
        if (UF[0] > UF[2])
            UF[2] = 0;
        else
            UF[0] = 0;
    }
    if (Un[0] == Un[3])
    {
        if (UF[0] > UF[3])
            UF[3] = 0;
        else
            UF[0] = 0;
    }
    if (Un[1] == Un[2])
    {
        if (UF[1] > UF[2])
            UF[2] = 0;
        else
            UF[1] = 0;
    }
    if (Un[1] == Un[3])
    {
        if (UF[1] > UF[3])
            UF[3] = 0;
        else
            UF[1] = 0;
    }
    if (Un[2] == Un[3])
    {
        if (UF[2] > UF[3])
            UF[3] = 0;
        else
            UF[2] = 0;
    }
    t1 = UF[0] * UFF[Un[0]];
    t2 = UF[1] * UFF[Un[1]];
    t3 = UF[2] * UFF[Un[2]];
    t4 = UF[3] * UFF[Un[3]];
    temp1 = t1 + t2 + t3 + t4;
    temp2 = UF[0] + UF[1] + UF[2] + UF[3];
    U = temp1 / temp2;
    return U;
}