#ifndef BSP_PWM__
#define BSP_PWM__

#include "driver/ledc.h"
#include "hal/ledc_types.h"

void pwm_init(ledc_timer_t timer_num,ledc_timer_bit_t resolution,uint32_t freq,uint8_t pwm_io,ledc_channel_t channel,uint32_t pwm_duty,ledc_intr_type_t intr);
void pwm_close(ledc_channel_t channel,uint32_t idle_level);
void pwm_set_duty(ledc_channel_t channel,uint32_t duty);
void pwm_set_freq(ledc_timer_t timer_num,uint32_t freq);

#endif
