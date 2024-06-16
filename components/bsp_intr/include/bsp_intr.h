#ifndef BSP_INTR__
#define BSP_INTR__

#include "driver/gptimer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "PID.h"

extern myPID pitch_pid;
extern myPID roll_pid;
extern myPID yaw_pid;;
extern float pitch_target;
extern float roll_target;
extern float yaw_target;
extern float Pitch;
extern float Roll;
extern float Yaw;

extern SemaphoreHandle_t Sensor_get_data;
extern SemaphoreHandle_t Pid_Contrl;

void imu_timer_create(void);
void imu_intr_enable_and_start(void);
void imu_timer_stop(void);
void imu_timer_restart(void);
void control_timer_create(void);
void control_intr_enable_and_start(void);

#endif
