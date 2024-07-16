#ifndef INDICATOR__
#define INDICATOR__

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "bsp_spi.h"

#define INDICATOR_HOST               SPI_SECOND_HOST
#define INDICATOR_SPI_MODE           3    //CPOL 1   CPHA 1
#define indicator_io                 GPIO_NUM_2
#define INDICATOR_SPI_FRQ_HZ         6.4 * 1000000  //6.4MHz
#define BIT_0_H                      2      //bit
#define BIT_0_L                      6      //bit
#define BIT_1_H                      6      //bit
#define BIT_1_L                      2      //bit
#define BIT_RESET_L                  1600   //bit
#define BIT_0                        0xC0
#define BIT_1                        0xFC

typedef struct indicator_bre
{
  uint8_t Dir;
  uint8_t R,G,B;
  uint8_t cal_R,cal_G,cal_B;
  uint32_t Cnt;
  uint32_t Cnt_len;
}indicator_bre;

typedef enum
{
  Normal = 0,
  Breath,
} IND_Mode_e;

typedef struct indicator_queue_s
{
  uint8_t R;
  uint8_t G;
  uint8_t B;
  uint8_t Mode;
  uint32_t Bre_Len;
}IND_Message_t;

typedef struct
{
  indicator_bre Bre;
  spi_device_handle_t indicator;
  QueueHandle_t queue;
  IND_Message_t temp_Message;

  void (*init)(void* Indicator);
  void (*Color_Set)(void* Indicator, uint8_t R, uint8_t G, uint8_t B);
  void (*Bre_init)(void* Indicator, uint8_t R, uint8_t G, uint8_t B, uint32_t len);
  void (*Bre_Color_Set)(void* Indicator, uint8_t R, uint8_t G, uint8_t B, uint32_t len);
  void (*Bre_Cal)(void* Indicator);
  void (*Send_Message)(void* Indicator, uint8_t R, uint8_t G, uint8_t B, IND_Mode_e Mode, uint32_t Bre_Len);
  void (*Adjust)(void* Indicator);
}Indicator_Classdef;

void Indicator_Class_init(Indicator_Classdef* Indicator);
/*
void indicator_init(Indicator_Classdef* Indicator);
void indicator_set(Indicator_Classdef* Indicator, uint8_t R, uint8_t G, uint8_t B);
void indicator_breath_init(Indicator_Classdef* Indicator, indicator_bre* Bre, uint8_t R, uint8_t G, uint8_t B, uint32_t len);
void indicator_breath_set(Indicator_Classdef* Indicator, indicator_bre* Bre, uint8_t R, uint8_t G, uint8_t B, uint32_t len);
void indicator_breath_cal(Indicator_Classdef* Indicator, indicator_bre* Bre);
*/

#endif
