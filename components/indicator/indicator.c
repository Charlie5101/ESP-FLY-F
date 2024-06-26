#include <stdio.h>
#include "indicator.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

#define indicator_num           1
//Mutex Lock Handle
SemaphoreHandle_t Indicator_Mutex_Lock;
static const char* TAG = "indicator:";

//spi device handle
spi_device_handle_t indicator;

uint8_t reset_data[200] = {0};

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

void cal_signal_RGB_data(uint8_t R,uint8_t G,uint8_t B,uint8_t temp[])
{
  //G
  for(uint8_t i=0;i<8;i++)
  {
    if( ((G << i) & 0x80) == 0x80)
    {
      *(temp + i) = BIT_1;
    }
    else
    {
      *(temp + i) = BIT_0;
    }
  }
  //R
  for(uint8_t i=0;i<8;i++)
  {
    if( ((R << i) & 0x80) == 0x80)
    {
      *(temp + 8 + i) = BIT_1;
    }
    else
    {
      *(temp + 8 + i) = BIT_0;
    }
  }
  //B
  for(uint8_t i=0;i<8;i++)
  {
    if( ((B << i) & 0x80) == 0x80)
    {
      *(temp + 16 + i) = BIT_1;
    }
    else
    {
      *(temp + 16 + i) = BIT_0;
    }
  }
}

void cal_RGB_data(uint8_t R,uint8_t G,uint8_t B,uint8_t temp[],uint8_t led_num)
{
  
}

void indicator_set_pixel(uint8_t data[])
{
  xSemaphoreTake(Indicator_Mutex_Lock,portMAX_DELAY);     //Lock
  uint8_t tt[24] = {0};
  //ESP_LOGI(TAG,"set");
  spi_connect_start(INDICATOR_HOST,-1,&indicator,8 * 24 + 1600,data,NULL);
  // spi_connect_start(INDICATOR_HOST,-1,&indicator,sizeof(*data)/sizeof(data[0]) * 8 + 1600,data,NULL);
  xSemaphoreGive(Indicator_Mutex_Lock);     //UNLock
}

void indicator_data_add_reset(uint8_t out_data[24+200],uint8_t data[24])
{
  memcpy(out_data,reset_data,sizeof(reset_data));
  memcpy(out_data + sizeof(reset_data),data,24);
}

void indicator_set(uint8_t R,uint8_t G,uint8_t B)
{
  uint8_t temp[24];
  uint8_t data[24+sizeof(reset_data)];
  cal_signal_RGB_data(R,G,B,temp);
  indicator_data_add_reset(data,temp);
  indicator_set_pixel(data);
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
