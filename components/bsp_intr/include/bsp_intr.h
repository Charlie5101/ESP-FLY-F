#ifndef BSP_INTR__
#define BSP_INTR__

#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

extern SemaphoreHandle_t Sensor_get_data;

void imu_timer_create(void);
bool imu_data_get_intr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx);
void intr_all_enable_and_start(void);
void imu_timer_restart(void);

#endif
