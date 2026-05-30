#ifndef _CROSS_H_
#define _CROSS_H_

#include "head.h"


void Cross_Detect();
void calculateSlopeIntercept(int startY, int endY, 
                            uint8 border[TRACK_H],
                            float& slope, 
                            float& intercept);

#endif