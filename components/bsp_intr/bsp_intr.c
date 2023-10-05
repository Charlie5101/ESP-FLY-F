#include <stdio.h>
#include "bsp_intr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
// #include "freertos/task.h"

SemaphoreHandle_t Sensor_get_data;

gptimer_handle_t imu_timer = NULL;

void imu_timer_create(void)
{
  gptimer_config_t imu_timer_cfg = {
    .clk_src = GPTIMER_CLK_SRC_DEFAULT,
    .direction = GPTIMER_COUNT_UP,
    .resolution_hz = 20 * 1000 * 1000,
    .flags.intr_shared = false,
  };
  ESP_ERROR_CHECK(gptimer_new_timer(&imu_timer_cfg,&imu_timer));

  gptimer_alarm_config_t imu_timer_alarm_cfg = {
    .alarm_count = 940,
    .reload_count = 0,
    .flags.auto_reload_on_alarm = true,
  };
  ESP_ERROR_CHECK(gptimer_set_alarm_action(imu_timer,&imu_timer_alarm_cfg));

  gptimer_event_callbacks_t imu_cbs = {
    .on_alarm = imu_data_get_intr,
  };
  gptimer_register_event_callbacks(imu_timer,&imu_cbs,NULL);
}

bool imu_data_get_intr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
  gptimer_stop(imu_timer);
  //Send Semphr to read imu
  xSemaphoreGiveFromISR(Sensor_get_data,NULL);
  return 0;
}

void intr_all_enable_and_start(void)
{
  Sensor_get_data = xSemaphoreCreateBinary();
  gptimer_enable(imu_timer);

  gptimer_start(imu_timer);
}

void imu_timer_restart(void)
{
  gptimer_set_raw_count(imu_timer,0);
  // gptimer_enable(imu_timer);
  gptimer_start(imu_timer);
}
