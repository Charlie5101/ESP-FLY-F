#include <stdio.h>
#include "bsp_pwm.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

//Mutex Lock Handle
SemaphoreHandle_t PWM_CH0_Mutex_Lock;
SemaphoreHandle_t PWM_CH1_Mutex_Lock;
SemaphoreHandle_t PWM_CH2_Mutex_Lock;
SemaphoreHandle_t PWM_CH3_Mutex_Lock;
SemaphoreHandle_t PWM_CH4_Mutex_Lock;
SemaphoreHandle_t PWM_CH5_Mutex_Lock;
SemaphoreHandle_t PWM_CH6_Mutex_Lock;
SemaphoreHandle_t PWM_CH7_Mutex_Lock;

void pwm_init(ledc_timer_t timer_num,ledc_timer_bit_t resolution,uint32_t freq,uint8_t pwm_io,ledc_channel_t channel,uint32_t pwm_duty,ledc_intr_type_t intr)
{
  //Mutex Lock
  PWM_CH0_Mutex_Lock = xSemaphoreCreateMutex();
  PWM_CH1_Mutex_Lock = xSemaphoreCreateMutex();
  PWM_CH2_Mutex_Lock = xSemaphoreCreateMutex();
  PWM_CH3_Mutex_Lock = xSemaphoreCreateMutex();
  PWM_CH4_Mutex_Lock = xSemaphoreCreateMutex();
  PWM_CH5_Mutex_Lock = xSemaphoreCreateMutex();
  PWM_CH6_Mutex_Lock = xSemaphoreCreateMutex();
  PWM_CH7_Mutex_Lock = xSemaphoreCreateMutex();
  if(PWM_CH0_Mutex_Lock == NULL)
  {
    ESP_LOGE("PWM","Ch0 Mutex Lock Create Fail......");
  }
  if(PWM_CH1_Mutex_Lock == NULL)
  {
    ESP_LOGE("PWM","Ch1 Mutex Lock Create Fail......");
  }
  if(PWM_CH2_Mutex_Lock == NULL)
  {
    ESP_LOGE("PWM","Ch2 Mutex Lock Create Fail......");
  }
  if(PWM_CH3_Mutex_Lock == NULL)
  {
    ESP_LOGE("PWM","Ch3 Mutex Lock Create Fail......");
  }
  if(PWM_CH4_Mutex_Lock == NULL)
  {
    ESP_LOGE("PWM","Ch4 Mutex Lock Create Fail......");
  }
  if(PWM_CH5_Mutex_Lock == NULL)
  {
    ESP_LOGE("PWM","Ch5 Mutex Lock Create Fail......");
  }
  if(PWM_CH6_Mutex_Lock == NULL)
  {
    ESP_LOGE("PWM","Ch6 Mutex Lock Create Fail......");
  }
  if(PWM_CH7_Mutex_Lock == NULL)
  {
    ESP_LOGE("PWM","Ch7 Mutex Lock Create Fail......");
  }

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
  //Mutex Lock
  switch(channel)
  {
  case LEDC_CHANNEL_0:
    xSemaphoreTake(PWM_CH0_Mutex_Lock,portMAX_DELAY);     //Lock
    break;
  case LEDC_CHANNEL_1:
    xSemaphoreTake(PWM_CH1_Mutex_Lock,portMAX_DELAY);
    break;
  case LEDC_CHANNEL_2:
    xSemaphoreTake(PWM_CH2_Mutex_Lock,portMAX_DELAY);
    break;
  case LEDC_CHANNEL_3:
    xSemaphoreTake(PWM_CH3_Mutex_Lock,portMAX_DELAY);
    break;
  case LEDC_CHANNEL_4:
    xSemaphoreTake(PWM_CH4_Mutex_Lock,portMAX_DELAY);
    break;
  case LEDC_CHANNEL_5:
    xSemaphoreTake(PWM_CH5_Mutex_Lock,portMAX_DELAY);
    break;
  case LEDC_CHANNEL_6:
    xSemaphoreTake(PWM_CH6_Mutex_Lock,portMAX_DELAY);
    break;
  case LEDC_CHANNEL_7:
    xSemaphoreTake(PWM_CH7_Mutex_Lock,portMAX_DELAY);
    break;
  default:
    break;
  }

  ledc_set_duty(LEDC_LOW_SPEED_MODE,channel,duty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE,channel);

  //Mutex UNLock
  switch(channel)
  {
  case LEDC_CHANNEL_0:
    xSemaphoreGive(PWM_CH0_Mutex_Lock);     //UNLock
    break;
  case LEDC_CHANNEL_1:
    xSemaphoreGive(PWM_CH1_Mutex_Lock);
    break;
  case LEDC_CHANNEL_2:
    xSemaphoreGive(PWM_CH2_Mutex_Lock);
    break;
  case LEDC_CHANNEL_3:
    xSemaphoreGive(PWM_CH3_Mutex_Lock);
    break;
  case LEDC_CHANNEL_4:
    xSemaphoreGive(PWM_CH4_Mutex_Lock);
    break;
  case LEDC_CHANNEL_5:
    xSemaphoreGive(PWM_CH5_Mutex_Lock);
    break;
  case LEDC_CHANNEL_6:
    xSemaphoreGive(PWM_CH6_Mutex_Lock);
    break;
  case LEDC_CHANNEL_7:
    xSemaphoreGive(PWM_CH7_Mutex_Lock);
    break;
  default:
    break;
  }
}

void pwm_set_freq(ledc_timer_t timer_num,uint32_t freq)
{
  ledc_set_freq(LEDC_LOW_SPEED_MODE,timer_num,freq);
}
