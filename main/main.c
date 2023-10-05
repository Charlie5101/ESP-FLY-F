#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include "indicator.h"
#include "sensor.h"
#include "black_box.h"
#include "motor.h"
#include "bsp_gpio.h"
#include "bsp_intr.h"

/*task config*/
#define Task_MAIN_Stack       1024
#define Task_MAIN_Prio        3
#define Task_sensor_Stack     4096
#define Task_sensor_Prio      3
#define Task_black_box_Stack  4096
#define Task_black_box_Prio   3
#define Task_indicator_Stack  4096
#define Task_indicator_Prio   3
#define Task_buzzer_Stack     4096
#define Task_buzzer_Prio      3
#define Task_bat_adc_Stack    4096
#define Task_bat_adc_Prio     3
#define Task_motor_Stack      4096
#define Task_motor_Prio       3
#define Task_GPS_Stack        4096
#define Task_GPS_Prio         3
#define Task_RGB_LED_Stack    4096
#define Task_RGB_LED_Prio     3
#define Task_receiver_Stack   4096
#define Task_receiver_Prio    3

/*ground task*/
#define Task_IIC_Stack        4096
#define Task_IIC_Prio         3
#define Task_Uart_Stack       4096
#define Task_Uart_Prio        3
#define Task_SPI_Stack        4096
#define Task_SPI_Prio         3
//PWM control write in interrupt

/*component config*/
#define SENSOR_ENABLE    1
#define BLACK_BOX_ENABLE 1
#define INDICATOR_ENABLE 1
#define BUZZER_ENABLE    1
#define BAT_ADC_ENABLE   1
#define MOTOR_ENABLE     1
#define GPS_ENABLE       1
#define RGB_LED_ENABLE   1
#define RECEIVER_ENABLE  1

/*Task Handle*/
TaskHandle_t MAIN_Handle;
TaskHandle_t Sensor_Handle;
TaskHandle_t Black_box_Handle;
TaskHandle_t Indicator_Handle;
TaskHandle_t Buzzer_Handle;
TaskHandle_t Bat_adc_Handle;
TaskHandle_t Motor_Handle;
TaskHandle_t GPS_Handle;
TaskHandle_t RGB_LED_Handle;
TaskHandle_t Receiver_HANDLE;

/*Task fun declear*/
void Task_MAIN(void *arg);
void Task_sensor(void *arg);
void Task_black_box(void *arg);
void Task_indicator(void *arg);
void Task_buzzer(void *arg);
void Task_bat_adc(void *arg);
void Task_motor(void *arg);
void Task_GPS(void *arg);
void Task_RGB_LED(void *arg);
void Task_receiver(void *arg);

uint8_t app_main(void)
{
  //Allow other core to finish initialization
  vTaskDelay(1000);

  //peripherals init
  spi_bus_init(SPI_MOSI,SPI_MISO,SPI_SCLK);
  //Create semaphores

  //Create tasks
  xTaskCreatePinnedToCore(Task_MAIN,      "main task",      Task_MAIN_Stack,      NULL, Task_MAIN_Prio,       &MAIN_Handle,      0);
  #if SENSOR_ENABLE == 1
  xTaskCreatePinnedToCore(Task_sensor,    "IMU task",       Task_sensor_Stack,    NULL, Task_sensor_Prio,     &Sensor_Handle,    1);
  #endif
  #if BLACK_BOX_ENABLE == 1
  xTaskCreatePinnedToCore(Task_black_box, "ram black box",  Task_black_box_Stack, NULL, Task_black_box_Prio,  &Black_box_Handle, 0);
  #endif
  #if INDICATOR_ENABLE == 1
  xTaskCreatePinnedToCore(Task_indicator, "indicator LED",  Task_indicator_Stack, NULL, Task_indicator_Prio,  &Indicator_Handle, 0);
  #endif
  #if BUZZER_ENABLE == 1
  xTaskCreatePinnedToCore(Task_buzzer,    "buzzer",         Task_buzzer_Stack,    NULL, Task_buzzer_Prio,     &Buzzer_Handle,    0);
  #endif
  #if BAT_ADC_ENABLE == 1
  xTaskCreatePinnedToCore(Task_bat_adc,   "Battery voltage",Task_bat_adc_Stack,   NULL, Task_bat_adc_Prio,    &Bat_adc_Handle,   0);
  #endif
  #if MOTOR_ENABLE == 1
  xTaskCreatePinnedToCore(Task_motor,     "PWM motor out",  Task_motor_Stack,     NULL, Task_motor_Prio,      &Motor_Handle,     0);
  #endif
  #if GPS_ENABLE == 1
  xTaskCreatePinnedToCore(Task_GPS,       "GPS/BDS",        Task_GPS_Stack,       NULL, Task_GPS_Prio,        &GPS_Handle,       0);
  #endif
  #if RGB_LED_ENABLE == 1
  xTaskCreatePinnedToCore(Task_RGB_LED,   "RGB LED out",    Task_RGB_LED_Stack,   NULL, Task_RGB_LED_Prio,    &RGB_LED_Handle,   0);
  #endif
  #if RECEIVER_ENABLE == 1
  xTaskCreatePinnedToCore(Task_receiver,   "Receiver",      Task_receiver_Stack,  NULL, Task_receiver_Prio,   &Receiver_HANDLE,  0);
  #endif

  return 0;
}

/**
 * @brief 
 * 
 * @param arg 
 */
void Task_MAIN(void *arg)
{

  for(;;)
  {
    vTaskDelay(10);
  }
}

/**
 * @brief 
 * 
 * @param arg 
 */
void Task_sensor(void *arg)
{
  sensor_init();
  float Ax,Ay,Az,Gx,Gy,Gz;
  imu_timer_create();
  vTaskDelay(1000);
  intr_all_enable_and_start();
  for(;;)
  {

    // ICM_42688P_read_Temp();
    // ICM_42688P_read_ACC(&Ax,&Ay,&Az);
    // ICM_42688P_read_GYRO(&Gx,&Gy,&Gz);
    xSemaphoreTake(Sensor_get_data,portMAX_DELAY);
    ICM_42688P_read_ACC_GYRO(&Ax,&Ay,&Az,&Gx,&Gy,&Gz);
    imu_timer_restart();
    // ESP_LOGI("Sensor","ICM-42688P Az: %f",Az);
    // ICM_42688P_read_FIFO();
    // vTaskDelay(1);
    // xSemaphoreTake(Sensor_get_data,portMAX_DELAY);
  }
}

/**
 * @brief 
 * 
 * @param arg 
 */
void Task_black_box(void *arg)
{
  black_box_init();
  for(;;)
  {
    vTaskDelay(10);
  }
}

/**
 * @brief WS2812 on board Task
 * 
 * @param arg 
 */
void Task_indicator(void *arg)
{
  indicator_init();
  vTaskDelay(10);
  indicator_set(0,80,0);
  vTaskDelay(250);
  static uint8_t B_t = 0;
  static uint8_t B_dir = 0;
  for(;;)
  {
    switch (B_dir)
    {
    case 0:
      if(B_t < 255)
      {
        indicator_set(0,0,B_t);
        B_t++;
      }
      else
      {
        B_dir = 1;
      }
      break;
    case 1:
      if(B_t > 0)
      {
        indicator_set(0,0,B_t);
        B_t--;
      }
      else
      {
        B_dir = 0;
      }
      break;
    case 2:
    default:
      break;
    }
    vTaskDelay(5);
  }
}

/**
 * @brief 
 * 
 * @param arg 
 */
void Task_buzzer(void *arg)
{
  for(;;)
  {
    vTaskDelay(10);
  }
}

/**
 * @brief 
 * 
 * @param arg 
 */
void Task_bat_adc(void *arg)
{
  for(;;)
  {
    vTaskDelay(10);
  }
}

/**
 * @brief 
 * 
 * @param arg 
 */
void Task_motor(void *arg)
{
  motor_init();
  motor_throttle_set(1,100.0);
  motor_throttle_set(2,75.0);
  motor_throttle_set(3,50.0);
  motor_throttle_set(4,25.0);
  servo_out_set(1,23.0);
  servo_out_set(2,35.0);
  servo_out_set(3,67.0);
  servo_out_set(4,89.0);
  for(;;)
  {
    vTaskDelay(10);
  }
}

/**
 * @brief 
 * 
 * @param arg 
 */
void Task_GPS(void *arg)
{
  for(;;)
  {
    vTaskDelay(10);
  }
}

/**
 * @brief 
 * 
 * @param arg 
 */
void Task_RGB_LED(void *arg)
{
  for(;;)
  {
    vTaskDelay(10);
  }
}

/**
 * @brief 
 * 
 * @param arg 
 */
void Task_receiver(void *arg)
{
  for(;;)
  {
    vTaskDelay(10);
  }
}

//intr
