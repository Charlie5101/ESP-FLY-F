#include <stdio.h>
#include "indicator.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define indicator_num           1

//Mutex Lock Handle
SemaphoreHandle_t Indicator_Mutex_Lock;

static const char* TAG = "indicator:";
led_strip_t *indicator = NULL;

void indicator_init()
{
  Indicator_Mutex_Lock = xSemaphoreCreateMutex();
  if(Indicator_Mutex_Lock == NULL)
  {
    ESP_LOGE("Indicator","Mutex Lock Create Fail......");
  }
  xSemaphoreTake(Indicator_Mutex_Lock,portMAX_DELAY);     //Lock

  ESP_LOGI(TAG,"Init");
  rmt_init(indicator_io,indicator_channel);
  led_strip_config_t strip_cfg = LED_STRIP_DEFAULT_CONFIG(indicator_num,indicator_channel);
  indicator = led_strip_new_rmt_ws2812(&strip_cfg);
  if (!indicator) {
        ESP_LOGE(TAG, "install WS2812 driver failed");
    }
  //close indicator
  ESP_ERROR_CHECK(indicator->clear(indicator, 100));

  xSemaphoreGive(Indicator_Mutex_Lock);     //UNLock
}

void indicator_set(uint8_t R,uint8_t G,uint8_t B)
{
  xSemaphoreTake(Indicator_Mutex_Lock,portMAX_DELAY);     //Lock

  //ESP_LOGI(TAG,"set");
  indicator->set_pixel(indicator,0,R,G,B);
  indicator->refresh(indicator,20);

  xSemaphoreGive(Indicator_Mutex_Lock);     //UNLock
}