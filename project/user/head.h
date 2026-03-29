//头文件，写好的库函数和宏定义调用都放在这里方便全局调用
#include "zf_common_headfile.hpp"

#include "../code/hardware/motor.h"
#include "../code/hardware/imu.h"
#include "../code/hardware/button.h"
#include "../code/hardware/beep.h"
#include "../code/hardware/encoder.h"
#include "../code/hardware/ips200.h"
#include "../code/hardware/trackuvc.h"
#include "../code/hardware/aiuvc.h"
#include "../code/vision/edge_extract.h"
#include "../code/vision/Threshold.h"
#include "../code/control/control.h"
#include "../code/control/PID.h"
#include "../code/control/Fuzzy.h"
#include "../code/control/TCPDebug.h"
#include "../code/tflite/aitask.h"
#include "../code/global_state.h"
#include "../code/element/cross.h"
#include "../code/element/island.h"

// #include "ramp.h"
// 
// #include "servo.h"
// 
// #include "brushless.h" 
// #include "barrier.h"


#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>
#include <map>
#include <array>
#include <math.h>
#include <string.h>
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <sstream>
#include <cctype>
#include <stdlib.h>

//定义TCP服务器地址
#define SERVER_IP "192.168.1.108" 
#define PORT_VOFA 1347
#define PORT_SEEKFREE 8086

#define MAX_DUTY  (60 )   // 最大 MAX_DUTY% 占空比

//定义巡线摄像头参数
#define TRACK_W 160
#define TRACK_H 120

#define AI_W 480
#define AI_H 640