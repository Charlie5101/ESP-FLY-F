#ifndef BSP_GPIO__
#define BSP_GPIO__

#include "driver/gpio.h"

void gpio_init_single(uint8_t io_num,gpio_mode_t io_mode,gpio_pullup_t pull_up,gpio_pulldown_t pull_down,gpio_int_type_t interrupt);


#endif
