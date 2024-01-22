#ifndef INDICATOR__
#define INDICATOR__

#include "bsp_rmt.h"

#define indicator_io                  GPIO_NUM_2
#define indicator_channel             RMT_CHANNEL_0

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
