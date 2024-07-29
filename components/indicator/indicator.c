#include <stdio.h>
#include "indicator.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

#define indicator_num           1
#define IND_QUEUE_LEN           5

void indicator_init(Indicator_Classdef* Indicator);
void indicator_set(Indicator_Classdef* Indicator, uint8_t R, uint8_t G, uint8_t B);
void indicator_breath_init(Indicator_Classdef* Indicator, uint8_t R, uint8_t G, uint8_t B, uint32_t len);
void indicator_breath_set(Indicator_Classdef* Indicator, uint8_t R, uint8_t G, uint8_t B, uint32_t len);
void indicator_breath_cal(Indicator_Classdef* Indicator);
void Indicator_Send_Message(Indicator_Classdef* Indicator, uint8_t R, uint8_t G, uint8_t B, IND_Mode_e Mode, uint32_t Bre_Len);
void Indicator_Adjust(Indicator_Classdef* Indicator);

//Mutex Lock Handle
SemaphoreHandle_t Indicator_Mutex_Lock;
static const char* TAG = "indicator:";

uint8_t reset_data[200] = {0};

void indicator_init(Indicator_Classdef* Indicator)
{
  Indicator_Mutex_Lock = xSemaphoreCreateMutex();

  if(Indicator_Mutex_Lock == NULL)
  {
    ESP_LOGE("Indicator","Mutex Lock Create Fail......");
  }
  xSemaphoreTake(Indicator_Mutex_Lock,portMAX_DELAY);     //Lock

  ESP_LOGI(TAG,"Init");
  
  spi_reg_device_to_bus(SPI_SECOND_HOST,6,&Indicator->indicator,INDICATOR_SPI_FRQ_HZ,INDICATOR_SPI_MODE);
  
  if (!Indicator->indicator) {
        ESP_LOGE(TAG, "install WS2812 driver failed");
    }

  xSemaphoreGive(Indicator_Mutex_Lock);     //UNLock
}

void cal_signal_RGB_data(Indicator_Classdef* Indicator, uint8_t R, uint8_t G, uint8_t B, uint8_t temp[])
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

void cal_RGB_data(Indicator_Classdef* Indicator, uint8_t R, uint8_t G, uint8_t B, uint8_t temp[], uint8_t led_num)
{
  
}

void indicator_set_pixel(Indicator_Classdef* Indicator, uint8_t data[])
{
  xSemaphoreTake(Indicator_Mutex_Lock,portMAX_DELAY);     //Lock
  //ESP_LOGI(TAG,"set");
  spi_connect_start(INDICATOR_HOST,-1,&Indicator->indicator,8 * 24 + 1600,data,NULL);
  // spi_connect_start(INDICATOR_HOST,-1,&indicator,sizeof(*data)/sizeof(data[0]) * 8 + 1600,data,NULL);
  xSemaphoreGive(Indicator_Mutex_Lock);     //UNLock
}

void indicator_data_add_reset(Indicator_Classdef* Indicator, uint8_t out_data[24+200], uint8_t data[24])
{
  memcpy(out_data,reset_data,sizeof(reset_data));
  memcpy(out_data + sizeof(reset_data),data,24);
}

void indicator_set(Indicator_Classdef* Indicator, uint8_t R, uint8_t G, uint8_t B)
{
  uint8_t temp[24];
  uint8_t data[24+sizeof(reset_data)];
  cal_signal_RGB_data(Indicator, R, G, B, temp);
  indicator_data_add_reset(Indicator, data, temp);
  indicator_set_pixel(Indicator, data);
}

void indicator_breath_init(Indicator_Classdef* Indicator, uint8_t R, uint8_t G, uint8_t B, uint32_t len)
{
  Indicator->Bre.Dir = 0;
  Indicator->Bre.Cnt = 0;
  Indicator->Bre.R = R;
  Indicator->Bre.G = G;
  Indicator->Bre.B = B;
  Indicator->Bre.Cnt_len = len;
}

void indicator_breath_set(Indicator_Classdef* Indicator, uint8_t R, uint8_t G, uint8_t B, uint32_t len)
{
  Indicator->Bre.Dir = 0;
  Indicator->Bre.Cnt = 0;
  Indicator->Bre.R = R;
  Indicator->Bre.G = G;
  Indicator->Bre.B = B;
  Indicator->Bre.Cnt_len = len;
}

void indicator_breath_cal(Indicator_Classdef* Indicator)
{
  switch (Indicator->Bre.Dir)
  {
    case 0:
      if(Indicator->Bre.Cnt < Indicator->Bre.Cnt_len)
      {
        Indicator->Bre.cal_R = (uint8_t)(Indicator->Bre.R * ((float)Indicator->Bre.Cnt / (float)Indicator->Bre.Cnt_len));
        Indicator->Bre.cal_G = (uint8_t)(Indicator->Bre.G * ((float)Indicator->Bre.Cnt / (float)Indicator->Bre.Cnt_len));
        Indicator->Bre.cal_B = (uint8_t)(Indicator->Bre.B * ((float)Indicator->Bre.Cnt / (float)Indicator->Bre.Cnt_len));
        indicator_set(Indicator, Indicator->Bre.cal_R, Indicator->Bre.cal_G, Indicator->Bre.cal_B);
        Indicator->Bre.Cnt++;
      }
      else
      {
        Indicator->Bre.Dir = 1;
      }
      break;
    case 1:
      if(Indicator->Bre.Cnt > 0)
      {
        Indicator->Bre.cal_R = (uint8_t)(Indicator->Bre.R * ((float)Indicator->Bre.Cnt / (float)Indicator->Bre.Cnt_len));
        Indicator->Bre.cal_G = (uint8_t)(Indicator->Bre.G * ((float)Indicator->Bre.Cnt / (float)Indicator->Bre.Cnt_len));
        Indicator->Bre.cal_B = (uint8_t)(Indicator->Bre.B * ((float)Indicator->Bre.Cnt / (float)Indicator->Bre.Cnt_len));
        indicator_set(Indicator, Indicator->Bre.cal_R, Indicator->Bre.cal_G, Indicator->Bre.cal_B);
        Indicator->Bre.Cnt--;
      }
      else
      {
        Indicator->Bre.Dir = 0;
      }
      break;
    case 2:
    default:
      break;
  }
}

void Indicator_Class_init(Indicator_Classdef* Indicator)
{
  bzero(&Indicator->Bre, 11);

  Indicator->init = (void (*)(void*))indicator_init;
  Indicator->Color_Set = (void (*)(void*, uint8_t R, uint8_t G, uint8_t B))indicator_set;
  Indicator->Bre_init = (void (*)(void*, uint8_t R, uint8_t G, uint8_t B, uint32_t len))indicator_breath_init;
  Indicator->Bre_Color_Set = (void (*)(void*, uint8_t R, uint8_t G, uint8_t B, uint32_t len))indicator_breath_set;
  Indicator->Bre_Cal = (void (*)(void*))indicator_breath_cal;
  Indicator->Send_Message = (void (*)(void*, uint8_t R, uint8_t G, uint8_t B, IND_Mode_e Mode, uint32_t Bre_Len))Indicator_Send_Message;
  Indicator->Adjust = (void (*)(void*))Indicator_Adjust;

  Indicator->queue = xQueueCreate(IND_QUEUE_LEN, sizeof(IND_Message_t));
  if(Indicator->queue == NULL)
  {
    ESP_LOGW(TAG,"Indicator Queue Create Fail......");
  }

  Indicator->init(Indicator);
}

void Indicator_Send_Message(Indicator_Classdef* Indicator, uint8_t R, uint8_t G, uint8_t B, IND_Mode_e Mode, uint32_t Bre_Len)
{
  IND_Message_t Message = {
    .R = R,
    .G = G,
    .B = B,
    .Mode = Mode,
    .Bre_Len = Bre_Len,
  };
  xQueueSend(Indicator->queue, &Message, 0);
}

void Indicator_Adjust(Indicator_Classdef* Indicator)
{
  if(xQueueReceive(Indicator->queue, &Indicator->temp_Message, 0) == pdTRUE)
  {
    if(Indicator->temp_Message.Mode == Normal)
    {
      Indicator->Color_Set(Indicator, Indicator->temp_Message.R, Indicator->temp_Message.G, Indicator->temp_Message.B);
    }
    else if(Indicator->temp_Message.Mode == Breath)
    {
      Indicator->Bre_Color_Set(Indicator, Indicator->temp_Message.R, Indicator->temp_Message.G, Indicator->temp_Message.B, Indicator->temp_Message.Bre_Len);
    }
  }
  else
  {
    if(Indicator->temp_Message.Mode == Normal)
    {
    }
    else if(Indicator->temp_Message.Mode == Breath)
    {
      Indicator->Bre_Cal(Indicator);
    }
  }
}
