#include "bsp_rmt.h"
#include "led_strip.h"

#define indicator_io                  GPIO_NUM_2
#define indicator_channel             RMT_CHANNEL_0

void indicator_init();
void indicator_set(uint8_t R,uint8_t G,uint8_t B);
