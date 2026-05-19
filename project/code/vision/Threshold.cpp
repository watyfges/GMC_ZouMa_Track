#include "head.h"//阈值处理模块，按需调用

int Max_01_value = 130;//大津法最大阈值限制

extern uint8_t img_gray[TRACK_H][TRACK_W];
extern uint8_t img_bin[TRACK_H][TRACK_W];
/*-------------------------------------------------------------------------------------------------------------------
  @brief     sobel二值化
  @param     imagein       原图数组
             imangeout     二值化后的数组
  @return    null
  Sample     lq_sobelAutoThreshold (mt9v03x_image[MT9V03X_H][MT9V03X_W],image_two_value[MT9V03X_H][MT9V03X_W])
  @note      会比大津慢一些，效果比大津法好不少
-------------------------------------------------------------------------------------------------------------------*/
void lq_sobelAutoThreshold(uint8 imageIn[][TRACK_W], uint8 imageOut[][TRACK_W])//sobel算子自动阈值处理
{
    // 增加边界处理范围
    short xStart = 1;
    short xEnd = TRACK_W - 1;
    short yStart = 1;
    short yEnd = TRACK_H - 1;
    
    for (int i = yStart; i < yEnd; i++)
    {
        for (int j = xStart; j < xEnd; j++)
        {
            // Sobel算子 - 水平方向
            short gx = -(short)imageIn[i-1][j-1] - 2*(short)imageIn[i][j-1] - (short)imageIn[i+1][j-1]
                      + (short)imageIn[i-1][j+1] + 2*(short)imageIn[i][j+1] + (short)imageIn[i+1][j+1];
            
            // Sobel算子 - 垂直方向
            short gy = -(short)imageIn[i-1][j-1] - 2*(short)imageIn[i-1][j] - (short)imageIn[i-1][j+1]
                      + (short)imageIn[i+1][j-1] + 2*(short)imageIn[i+1][j] + (short)imageIn[i+1][j+1];
            
            // 计算梯度幅值 (近似：|Gx| + |Gy|)
            short gradient = abs(gx) + abs(gy);
            
            // 使用5x5邻域计算局部均值 (扩展邻域范围)
            long neighborhoodSum = 0;
            for (int m = -2; m <= 2; m++) {
                for (int n = -2; n <= 2; n++) {
                    neighborhoodSum += (short)imageIn[i+m][j+n];
                }
            }
            
            // 自适应动态阈值设置
            float localMean = neighborhoodSum / 25.0f; // 5x5均值
            float dynamicThreshold = (localMean * 0.5f) + 16; // 增强对比因子+最小阈值
            
            // 增强型二值化决策
            if (gradient > dynamicThreshold) {
                imageOut[i][j] = 0;    // 边缘 - 黑色
            } else {
                imageOut[i][j] = 0xFF; // 背景 - 白色
            }
        }
    }
}

/*************************************************************************
 *  函数名称：int GetOTSU(unsigned char tmImage[][UVC_WIDTH], unsigned char outImage[][UVC_WIDTH]) 
 *  功能说明：使用大津法（Otsu's method）计算图像的最佳二值化阈值
 *  参数说明：tmImage - 输入的灰度图像数据，维度为[LCDH][LCDW]
 *  函数返回：最佳阈值（0-255之间的整数值）
*************************************************************************/
// int GetOTSU(unsigned char tmImage[][UVC_WIDTH], unsigned char outImage[][UVC_WIDTH]) //大津法
// {
//     int LCDH = UVC_HEIGHT;
//     int LCDW = UVC_WIDTH;
//     short i, j;
//     unsigned long PixelCount = 0;           // 像素总数
//     unsigned long PixelSum = 0;              // 像素灰度总和
//     unsigned long ForegroundCount = 0;       // 前景像素计数
//     unsigned long ForegroundSum = 0;         // 前景像素灰度总和
//     float ForegroundRatio, BackgroundRatio;  // 前景/背景像素比例
//     float ForegroundMean, BackgroundMean;    // 前景/背景平均灰度
//     float MaxVariance = -1.0;                // 最大类间方差
//     short Threshold = 0;                     // 最佳阈值
//     unsigned int Histogram[256] = {0};       // 灰度直方图

//     // 统计图像直方图
//     for (j = 0; j < LCDH; j++) {
//         for (i = 0; i < LCDW; i++) {
//             Histogram[tmImage[j][i]]++;
//         }
//     }

//     // 确定有效灰度范围
//     short MinGray = 0;
//     short MaxGray = 255;
//     while (MinGray < 256 && Histogram[MinGray] == 0) MinGray++;
//     while (MaxGray > MinGray && Histogram[MaxGray] == 0) MaxGray--;

//     // 特殊情况处理
//     if (MaxGray == MinGray) return MinGray;      // 只有一种灰度
//     if (MaxGray == MinGray + 1) return MinGray;   // 只有两种灰度

//     // 计算总像素数和总灰度值
//     for (i = MinGray; i <= MaxGray; i++) {
//         PixelCount += Histogram[i];
//         PixelSum += Histogram[i] * i;
//     }

//     // 遍历所有可能的阈值
//     for (i = MinGray; i < MaxGray; i++) {
//         ForegroundCount += Histogram[i];
//         ForegroundSum += Histogram[i] * i;

//         // 计算前景/背景统计量
//         unsigned long BackgroundCount = PixelCount - ForegroundCount;
//         long BackgroundSum = PixelSum - ForegroundSum;

//         ForegroundRatio = ForegroundCount / (float)PixelCount;
//         BackgroundRatio = BackgroundCount / (float)PixelCount;
//         ForegroundMean = ForegroundSum / (float)ForegroundCount;
//         BackgroundMean = BackgroundSum / (float)BackgroundCount;

//         // 计算类间方差
//         float CurrentVariance = 
//             ForegroundRatio * BackgroundRatio * 
//             (ForegroundMean - BackgroundMean) * 
//             (ForegroundMean - BackgroundMean);

//         // 更新最大方差和最佳阈值
//         if (CurrentVariance > MaxVariance) {
//             MaxVariance = CurrentVariance;
//             Threshold = i;
//         }
//     }
//     for(int i = 0; i < UVC_HEIGHT; i++)
//     {
//         for(int j = 0; j < UVC_WIDTH; j++)
//         {
//             if (tmImage[i][j] < Threshold) {
//                 outImage[i][j] = 0;    // 边缘 - 黑色
//             } else {
//                 outImage[i][j] = 0xFF; // 背景 - 白色
//             }
//         }
//     }
//     return Threshold;
// }


//-------------------------------------------------------------------------------------------------------------------
//  @brief      优化的大津法
//  @param      image  图像数组
//  @param      clo    宽
//  @param      row    高
//  @param      pixel_threshold 阈值分离
//  @return     uint8
//  @since      2021.6.23
//  Sample usage:
//-------------------------------------------------------------------------------------------------------------------
uint8_t Threshold_deal(uint8_t* image,uint16_t col,uint16_t row,uint32_t pixel_threshold) {//大津法优化
#define GrayScale 256
  uint16_t width = col;
  uint16_t height = row;
  int pixelCount[GrayScale];
  float pixelPro[GrayScale];
  int i, j, pixelSum = width * height;
  uint8_t threshold = 0;
  uint8_t* data = image; //指向像素数据的指针
  for (i = 0; i < GrayScale; i++) {
    pixelCount[i] = 0;
    pixelPro[i] = 0;
  }

  uint32_t gray_sum = 0;
 //统计灰度级中每个像素在整幅图像中的个数
  for (i = 0; i < height; i += 1) {
    for (j = 0; j < width; j += 1) {

      pixelCount[(
          int)data[i * width + j]]++;   //将当前的点的像素值作为计数数组的下标
      gray_sum += (int)data[i * width + j]; //灰度值总和

    }
  }

  //计算每个像素值的点在整幅图像中的比例
  for (i = 0; i < GrayScale; i++) {
    pixelPro[i] = (float)pixelCount[i] / pixelSum;
  }


  //遍历灰度级[0,255]
  float w0, w1, u0tmp, u1tmp, u0, u1, u, deltaTmp, deltaMax = 0;
  w0 = w1 = u0tmp = u1tmp = u0 = u1 = u = deltaTmp = 0;
  for (j = 0; j < (int)pixel_threshold; j++) {
    w0 +=
        pixelPro[j];  //背景部分每个灰度值的像素点所占比例之和 即背景部分的比例
    u0tmp += j * pixelPro[j];  //背景部分 每个灰度值的点的比例 *灰度值

    w1 = 1 - w0;
    u1tmp = gray_sum / pixelSum - u0tmp;

    u0 = u0tmp / w0;    //背景平均灰度
    u1 = u1tmp / w1;    //前景平均灰度
    u = u0tmp + u1tmp;  //全局平均灰度
    deltaTmp = w0 * pow((u0 - u), 2) + w1 * pow((u1 - u), 2);
    if (deltaTmp > deltaMax) {
      deltaMax = deltaTmp;
      threshold = j;
    }
    if (deltaTmp < deltaMax) break;
  }
  return threshold;
}

void Get01change_dajin() {
  // int Threshold = Threshold_deal(img_gray[0], TRACK_W, TRACK_H, 180);
  // if (Threshold < Max_01_value)
  //   Threshold = Max_01_value;
  // uint8_t i, j = 0;
  // uint8_t thre;
  // for (i = 0; i < TRACK_H; i++) {
  //   for (j = 0; j < TRACK_W; j++) {
  //     if (j <= 15)
  //       thre = Threshold - 10;
  //     else if (j >= TRACK_W - 1 -15)
  //       thre = Threshold - 10;
  //     else
  //       thre = Threshold;

  //     if (img_gray[i][j] >(thre))         //数值越大，显示的内容越多，较浅的图像也能显示出来
  //       img_bin[i][j] = 255;  //白
  //     else
  //       img_bin[i][j] = 0;  //黑
  //   }
  // }
  lq_sobelAutoThreshold(img_gray, img_bin);
}

void Pixle_Filter()//像素滤波
{
  int nr;  //行
  int nc;  //列

  for (nr = 10; nr < 70; nr++) {
    for (nc = 10; nc < 150; nc = nc + 1) {
      if ((img_bin[nr][nc] == 0) && ((int)img_bin[nr - 1][nc] + (int)img_bin[nr + 1][nc] +
                                       (int)img_bin[nr][nc + 1] + (int)img_bin[nr][nc - 1] >=
                                   255 * 3)) {
        img_bin[nr][nc] = 1;
      }

    }
  }
}