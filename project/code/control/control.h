#ifndef CONTROL_H
#define CONTROL_H

#include "head.h"

void control_init();
void time_pit_callback();
void cleanup();
void pit_callback();
//void enc_pit_callback();
void IMU_pit_callback();

#endif // CONTROL_H