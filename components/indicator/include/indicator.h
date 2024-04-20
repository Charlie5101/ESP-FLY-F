#ifndef INDICATOR__
#define INDICATOR__

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

void indicator_init(void);
void indicator_set(uint8_t R,uint8_t G,uint8_t B);
void indicator_breath_init(indicator_bre* Bre,uint8_t R,uint8_t G,uint8_t B,uint32_t len);
void indicator_breath_set(indicator_bre* Bre,uint8_t R,uint8_t G,uint8_t B,uint32_t len);
void indicator_breath_cal(indicator_bre* Bre);

#endif
