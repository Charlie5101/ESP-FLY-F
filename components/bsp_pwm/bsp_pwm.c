#include <stdio.h>
#include "bsp_pwm.h"

void pwm_init(ledc_timer_t timer_num,ledc_timer_bit_t resolution,uint32_t freq,uint8_t pwm_io,ledc_channel_t channel,uint32_t pwm_duty,ledc_intr_type_t intr)
{
  ledc_timer_config_t pwm_conf = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .timer_num = timer_num,
    .clk_cfg = LEDC_APB_CLK,
    .duty_resolution = resolution,
    .freq_hz = freq,
  };
  ledc_timer_config(&pwm_conf);

  ledc_channel_config_t channel_conf = {
    .gpio_num = pwm_io,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = channel,
    .intr_type = intr,
    .timer_sel = timer_num,
    .duty = pwm_duty,
    .hpoint = 0,
  };
  ledc_channel_config(&channel_conf);
}

void pwm_close(ledc_channel_t channel,uint32_t idle_level)
{
  ledc_stop(LEDC_LOW_SPEED_MODE,channel,idle_level);
}

void pwm_set_duty(ledc_channel_t channel,uint32_t duty)
{
  ledc_set_duty(LEDC_LOW_SPEED_MODE,channel,duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE,channel);
}

void pwm_set_freq(ledc_timer_t timer_num,uint32_t freq)
{
  ledc_set_freq(LEDC_LOW_SPEED_MODE,timer_num,freq);
}
