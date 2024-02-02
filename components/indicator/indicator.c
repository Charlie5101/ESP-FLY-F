#include <stdio.h>
#include "indicator.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define indicator_num           1
//Mutex Lock Handle
SemaphoreHandle_t Indicator_Mutex_Lock;
static const char* TAG = "indicator:";

#if INDICATOR_MODE == RMT_INDICATOR

LED_handle_t *indicator = NULL;

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
  LED_handle_cfg_t strip_cfg = LED_handle_DEFAULT_CONFIG(indicator_num,indicator_channel);
  indicator = LED_handle_new_ws2812(&strip_cfg);
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

void indicator_breath_init(indicator_bre* Bre,uint8_t R,uint8_t G,uint8_t B,uint32_t len)
{
  Bre->Dir = 0;
  Bre->Cnt = 0;
  Bre->R = R;
  Bre->G = G;
  Bre->B = B;
  Bre->Cnt_len = len;
}

void indicator_breath_set(indicator_bre* Bre,uint8_t R,uint8_t G,uint8_t B,uint32_t len)
{
  Bre->Dir = 0;
  Bre->Cnt = 0;
  Bre->R = R;
  Bre->G = G;
  Bre->B = B;
  Bre->Cnt_len = len;
}

void indicator_breath_cal(indicator_bre* Bre)
{
  switch (Bre->Dir)
  {
    case 0:
      if(Bre->Cnt < Bre->Cnt_len)
      {
        Bre->cal_R = (uint8_t)(Bre->R * ((float)Bre->Cnt / (float)Bre->Cnt_len));
        Bre->cal_G = (uint8_t)(Bre->G * ((float)Bre->Cnt / (float)Bre->Cnt_len));
        Bre->cal_B = (uint8_t)(Bre->B * ((float)Bre->Cnt / (float)Bre->Cnt_len));
        indicator_set(Bre->cal_R,Bre->cal_G,Bre->cal_B);
        Bre->Cnt++;
      }
      else
      {
        Bre->Dir = 1;
      }
      break;
    case 1:
      if(Bre->Cnt > 0)
      {
        Bre->cal_R = (uint8_t)(Bre->R * ((float)Bre->Cnt / (float)Bre->Cnt_len));
        Bre->cal_G = (uint8_t)(Bre->G * ((float)Bre->Cnt / (float)Bre->Cnt_len));
        Bre->cal_B = (uint8_t)(Bre->B * ((float)Bre->Cnt / (float)Bre->Cnt_len));
        indicator_set(Bre->cal_R,Bre->cal_G,Bre->cal_B);
        Bre->Cnt--;
      }
      else
      {
        Bre->Dir = 0;
      }
      break;
    case 2:
    default:
      break;
  }
}

#endif

#if INDICATOR_MODE == SPI_INDICATOR

//spi device handle
spi_device_handle_t indicator;

uint8_t reset_data[175] = {0};

void indicator_init()
{
  Indicator_Mutex_Lock = xSemaphoreCreateMutex();
  if(Indicator_Mutex_Lock == NULL)
  {
    ESP_LOGE("Indicator","Mutex Lock Create Fail......");
  }
  xSemaphoreTake(Indicator_Mutex_Lock,portMAX_DELAY);     //Lock

  ESP_LOGI(TAG,"Init");
  
  spi_reg_device_to_bus(SPI_SECOND_HOST,6,&indicator,INDICATOR_SPI_FRQ_HZ,INDICATOR_SPI_MODE);
  
  if (!indicator) {
        ESP_LOGE(TAG, "install WS2812 driver failed");
    }
  //close indicator
  // ESP_ERROR_CHECK(indicator->clear(indicator, 100));

  xSemaphoreGive(Indicator_Mutex_Lock);     //UNLock
}

void cal_signal_RGB_data(uint8_t R,uint8_t G,uint8_t B,uint8_t temp[21])
{
  // uint8_t BIT_0[1] = 0xC0; //0b1100000 0
  // uint8_t BIT_1[1] = 0x06; //0b0000011 0
  uint32_t temp_color_data = 0;
  // uint8_t temp[7 * 24 / 8] = {0};
  //G
  for(uint8_t i=0;i<4;i++)
  {
    if( (G >> i & 0x01) == 0x01)
    {
      temp_color_data = (temp_color_data << (i * 8) | BIT_1) >> 1;
    }
    else
    {
      temp_color_data = (temp_color_data << (i * 8) | BIT_0) >> 1;
    }
  }
  for(uint8_t i=0;i<4;i++)
  {
    temp_color_data = temp_color_data >> (i * 8);
    temp[i] = (temp_color_data && 0xFF);
  }
  for(uint8_t i=4;i<8;i++)
  {
    if( (G >> i & 0x01) == 0x01)
    {
      temp_color_data = (temp_color_data << (i * 8) | BIT_1) >> 1;
    }
    else
    {
      temp_color_data = (temp_color_data << (i * 8) | BIT_0) >> 1;
    }
  }
  for(uint8_t i=4;i<7;i++)
  {
    temp_color_data = temp_color_data >> (i * 8);
    temp[i+4] = (temp_color_data && 0xFF);
  }
  //R
  for(uint8_t i=0;i<4;i++)
  {
    if( (R >> i & 0x01) == 0x01)
    {
      temp_color_data = (temp_color_data << (i * 8) | BIT_1) >> 1;
    }
    else
    {
      temp_color_data = (temp_color_data << (i * 8) | BIT_0) >> 1;
    }
  }
  for(uint8_t i=0;i<4;i++)
  {
    temp_color_data = temp_color_data >> (i * 8);
    temp[i+7] = (temp_color_data && 0xFF);
  }
  for(uint8_t i=4;i<8;i++)
  {
    if( (R >> i & 0x01) == 0x01)
    {
      temp_color_data = (temp_color_data << (i * 8) | BIT_1) >> 1;
    }
    else
    {
      temp_color_data = (temp_color_data << (i * 8) | BIT_0) >> 1;
    }
  }
  for(uint8_t i=4;i<7;i++)
  {
    temp_color_data = temp_color_data >> (i * 8);
    temp[i+11] = (temp_color_data && 0xFF);
  }
  //B
  for(uint8_t i=0;i<4;i++)
  {
    if( (B >> i & 0x01) == 0x01)
    {
      temp_color_data = (temp_color_data << (i * 8) | BIT_1) >> 1;
    }
    else
    {
      temp_color_data = (temp_color_data << (i * 8) | BIT_0) >> 1;
    }
  }
  for(uint8_t i=0;i<4;i++)
  {
    temp_color_data = temp_color_data >> (i * 8);
    temp[i+14] = (temp_color_data && 0xFF);
  }
  for(uint8_t i=4;i<8;i++)
  {
    if( (B >> i & 0x01) == 0x01)
    {
      temp_color_data = (temp_color_data << (i * 8) | BIT_1) >> 1;
    }
    else
    {
      temp_color_data = (temp_color_data << (i * 8) | BIT_0) >> 1;
    }
  }
  for(uint8_t i=4;i<7;i++)
  {
    temp_color_data = temp_color_data >> (i * 8);
    temp[i+18] = (temp_color_data && 0xFF);
  }
}

void indicator_set(uint8_t R,uint8_t G,uint8_t B)
{
  xSemaphoreTake(Indicator_Mutex_Lock,portMAX_DELAY);     //Lock

  //ESP_LOGI(TAG,"set");
  // indicator->set_pixel(indicator,0,R,G,B);
  // indicator->refresh(indicator,20);

  xSemaphoreGive(Indicator_Mutex_Lock);     //UNLock
}

void indicator_breath_init(indicator_bre* Bre,uint8_t R,uint8_t G,uint8_t B,uint32_t len)
{

}

void indicator_breath_set(indicator_bre* Bre,uint8_t R,uint8_t G,uint8_t B,uint32_t len)
{

}

void indicator_breath_cal(indicator_bre* Bre)
{
  
}

#endif
