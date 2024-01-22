#include "driver/rmt.h"
#include "led_strip.h"

#define LED_handle_t led_strip_t
#define LED_handle_cfg_t led_strip_config_t
#define LED_handle_DEFAULT_CONFIG LED_STRIP_DEFAULT_CONFIG
#define LED_handle_new_ws2812 led_strip_new_rmt_ws2812

void rmt_init(uint8_t rmt_io,rmt_channel_t rmt_channel);
