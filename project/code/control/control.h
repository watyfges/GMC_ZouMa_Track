#ifndef CONTROL_H
#define CONTROL_H

#include "head.h"

void control_init();
void time_pit_callback();
void cleanup();
void pit_callback();
void brushless_pit_callback();

#endif // CONTROL_H