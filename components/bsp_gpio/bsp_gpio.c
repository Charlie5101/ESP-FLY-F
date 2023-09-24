#include <stdio.h>
#include "bsp_gpio.h"

void gpio_init_single(uint8_t io_num,gpio_mode_t io_mode,gpio_pullup_t pull_up,gpio_pulldown_t pull_down,gpio_int_type_t interrupt)
{
  gpio_config_t gpio_conf = {
    .pin_bit_mask = (1ULL << io_num),
    .mode = io_mode,
    .pull_up_en = pull_up,
    .pull_down_en = pull_down,
    .intr_type = interrupt,
  };
  gpio_config(&gpio_conf);
}

