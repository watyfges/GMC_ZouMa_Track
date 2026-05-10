#include "aitask.h"

uint16_t img_ai[AI_H][AI_W] = {0};

int ai_fetch_new_frame(void) {
    ai_uvc_get_rgb_image(img_ai[0]); 
}

void ai_extract(void) {
    ai_fetch_new_frame(); 
}