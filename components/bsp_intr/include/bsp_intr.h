#ifndef BSP_INTR__
#define BSP_INTR__

#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

extern SemaphoreHandle_t Sensor_get_data;

void imu_timer_create(void);
void imu_intr_enable_and_start(void);
void imu_timer_stop(void);
void imu_timer_restart(void);
void control_timer_create(void);
void control_intr_enable_and_start(void);

#endif
