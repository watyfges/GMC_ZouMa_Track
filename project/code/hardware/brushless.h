#ifndef __BRUSHLESS_H__

#include "zf_common_headfile.hpp"

void brushless_init();
void set_brushless_pwm(uint16 duty);
void cleanup_brushless();

#endif // ENCODER_H