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
#include "../code/hardware/tof.h"
#include "../code/vision/edge_extract.h"
#include "../code/vision/Threshold.h"
#include "../code/control/control.h"
#include "../code/control/PID.h"
#include "../code/control/Fuzzy.h"
#include "../code/tflite/aitask.h"
#include "../code/global_state.h"
#include "../code/element/cross.h"
#include "../code/element/island.h"
#include "../code/communication/TCPDebug.h"
#include "../code/communication/usart.h"

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
#include <utility>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <errno.h>


using namespace cv;
using namespace std;