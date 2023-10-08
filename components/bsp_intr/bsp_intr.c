#include <stdio.h>
#include "bsp_intr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
// #include "freertos/task.h"
#include "PID.h"

SemaphoreHandle_t Sensor_get_data;

gptimer_handle_t imu_timer = NULL;
gptimer_handle_t control_timer = NULL;

IRAM_ATTR static bool imu_data_get_intr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx);
IRAM_ATTR static bool control_intr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx);

void imu_timer_create(void)
{
  gptimer_config_t imu_timer_cfg = {
    .clk_src = GPTIMER_CLK_SRC_DEFAULT,
    .direction = GPTIMER_COUNT_UP,
    .resolution_hz = 10 * 1000 * 1000,
    .flags.intr_shared = false,
  };
  ESP_ERROR_CHECK(gptimer_new_timer(&imu_timer_cfg,&imu_timer));

  gptimer_alarm_config_t imu_timer_alarm_cfg = {
    .alarm_count = 940 * 2,
    .reload_count = 0,
    .flags.auto_reload_on_alarm = true,
  };
  ESP_ERROR_CHECK(gptimer_set_alarm_action(imu_timer,&imu_timer_alarm_cfg));

  gptimer_event_callbacks_t imu_cbs = {
    .on_alarm = imu_data_get_intr,
  };
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(imu_timer,&imu_cbs,NULL));
}

static bool imu_data_get_intr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
  if(timer == imu_timer)
  {
    // ESP_ERROR_CHECK(gptimer_stop(imu_timer));
    //Send Semphr to read imu
    xSemaphoreGiveFromISR(Sensor_get_data,NULL);
  }
  return 0;
}

void imu_intr_enable_and_start(void)
{
  Sensor_get_data = xSemaphoreCreateBinary();
  ESP_ERROR_CHECK(gptimer_enable(imu_timer));
  ESP_ERROR_CHECK(gptimer_start(imu_timer));
}

void imu_timer_stop(void)
{
  ESP_ERROR_CHECK(gptimer_stop(imu_timer));
}

void imu_timer_restart(void)
{
  ESP_ERROR_CHECK(gptimer_set_raw_count(imu_timer,0));
  ESP_ERROR_CHECK(gptimer_start(imu_timer));
}

void control_timer_create(void)
{
  gptimer_config_t control_timer_cfg = {
    .clk_src = GPTIMER_CLK_SRC_DEFAULT,
    .direction = GPTIMER_COUNT_UP,
    .resolution_hz = 10 * 1000 * 1000,
    .flags.intr_shared = false,
  };
  ESP_ERROR_CHECK(gptimer_new_timer(&control_timer_cfg,&control_timer));

  gptimer_alarm_config_t control_timer_alarm_cfg = {
    .alarm_count = 1000,
    .reload_count = 0,
    .flags.auto_reload_on_alarm = true,
  };
  ESP_ERROR_CHECK(gptimer_set_alarm_action(control_timer,&control_timer_alarm_cfg));

  gptimer_event_callbacks_t control_cbs = {
    .on_alarm = control_intr,
  };
  ESP_ERROR_CHECK(gptimer_register_event_callbacks(control_timer,&control_cbs,NULL));
}

static bool control_intr(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
  //Control
  

  return 0;
}

void control_intr_enable_and_start(void)
{
  ESP_ERROR_CHECK(gptimer_enable(control_timer));
  ESP_ERROR_CHECK(gptimer_start(control_timer));
}
