#include "beep.h"

#define BEEP_PATH        ZF_GPIO_BEEP
zf_driver_gpio  beep(BEEP_PATH, O_RDWR);

void beep_on () {
    beep.set_level(1);
}

void beep_off () {
    beep.set_level(0);
}