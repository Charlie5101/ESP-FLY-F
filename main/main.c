#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "indicator.h"
#include "sensor.h"
#include "black_box.h"
#include "motor.h"
#include "bsp_gpio.h"
#include "bsp_intr.h"
#include "myWifi.h"
#include "Kalman.h"

/*task config*/
#define Task_MAIN_Stack       4096
#define Task_MAIN_Prio        3
#define Task_sensor_Stack     4096 * 3
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
#define Task_UpMonitor_Stack  4096
#define Task_UpMonitor_Prio   1
#define Task_Wifi_Recv_Stack  4096
#define Task_Wifi_Recv_Prio   1

/*ground task*/
#define Task_IIC_Stack        4096
#define Task_IIC_Prio         3
#define Task_Uart_Stack       4096
#define Task_Uart_Prio        3
#define Task_SPI_Stack        4096
#define Task_SPI_Prio         3
//PWM control write in interrupt

/*component config*/
#define SENSOR_ENABLE         1
#define BLACK_BOX_ENABLE      1
#define INDICATOR_ENABLE      1
#define BUZZER_ENABLE         1
#define BAT_ADC_ENABLE        1
#define MOTOR_ENABLE          1
#define GPS_ENABLE            1
#define RGB_LED_ENABLE        1
#define RECEIVER_ENABLE       1
#define UPMONITOR_ENABLE      1
#define WIFI_RECV_ENABLE      1

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
TaskHandle_t Receiver_Handle;
TaskHandle_t UpMonitor_Handle;
TaskHandle_t Wifi_Recv_Handle;

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
void Task_UpMonitor(void *arg);
void Task_Wifi_Recv(void *arg);

/*Var*/
float ICM_temp = 0.0f;
float BMI_temp = 0.0f;
float ICM_Ax,ICM_Ay,ICM_Az,ICM_Gx,ICM_Gy,ICM_Gz;
float BMI_Ax,BMI_Ay,BMI_Az,BMI_Gx,BMI_Gy,BMI_Gz;
float Yaw_angle = 0.0,Yaw_speed = 0.0;
float Yaw_angle_BMI = 0.0;
float t = 0.0001;
myKalman_2 Gz_Kalman_ICM;
myKalman_2 Gz_Kalman_BMI;
// myKalman_2 Gz_Kalman;
float Gz = 0.0;
float Yaw = 0.0;
float Yaw_Kalman_angle = 0.0;

float ICM_Gx_offset = 0.0;
float BMI_Gx_offset = 0.0;
float ICM_Gy_offset = 0.0;
float BMI_Gy_offset = 0.0;
float ICM_Gz_offset = 0.25278f;     //0.111362f
float BMI_Gz_offset = 0.11278;

indicator_bre indicator_Bre;

uint8_t app_main(void)
{
  //Allow other core to finish initialization
  vTaskDelay(1000);
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  //peripherals init
  spi_bus_init(SPI_MOSI,SPI_MISO,SPI_SCLK);
  //Create semaphores

  //Create tasks
  xTaskCreatePinnedToCore(Task_MAIN,      "main task",      Task_MAIN_Stack,      NULL, Task_MAIN_Prio,       &MAIN_Handle,      0);
  #if SENSOR_ENABLE == 1
  xTaskCreatePinnedToCore(Task_sensor,    "IMU task",       Task_sensor_Stack,    NULL, Task_sensor_Prio,     &Sensor_Handle,    1);
  #endif
  #if BLACK_BOX_ENABLE == 1
  xTaskCreatePinnedToCore(Task_black_box, "flash black box",Task_black_box_Stack, NULL, Task_black_box_Prio,  &Black_box_Handle, 0);
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
  xTaskCreatePinnedToCore(Task_receiver,   "Receiver",      Task_receiver_Stack,  NULL, Task_receiver_Prio,   &Receiver_Handle,  0);
  #endif
  #if UPMONITOR_ENABLE == 1
  xTaskCreatePinnedToCore(Task_UpMonitor,  "UpMonitor",     Task_UpMonitor_Stack, NULL, Task_UpMonitor_Prio,  &UpMonitor_Handle, 0);
  #endif
  #if WIFI_RECV_ENABLE == 1
  xTaskCreatePinnedToCore(Task_Wifi_Recv,  "Wifi Receive",  Task_Wifi_Recv_Stack, NULL, Task_Wifi_Recv_Prio,  &Wifi_Recv_Handle, 0);
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
  control_timer_create();
  vTaskDelay(1000);
  control_intr_enable_and_start();
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
  ICM_42688P_Get_Bais(&ICM_Gx_offset,&ICM_Gy_offset,&ICM_Gz_offset);
  BMI270_Get_Bais(&BMI_Gx_offset,&BMI_Gy_offset,&BMI_Gz_offset);
  Kalman_2_init(&Gz_Kalman_ICM,1.0,t,0.0,1.0,               //F
                           0.5*t*t,t,                   //B
                           1.0,0.0,0.0,1.0,             //H
                           0.1,0.0,0.0,0.1,             //Q
                           100.0,0.0,0.0,100.0,         //R
                           0.0,0.0,                     //X
                           0.0);                        //Ut
  Kalman_2_init(&Gz_Kalman_BMI, 1.0,t,0.0,1.0,               //F
                                0.5*t*t,t,                   //B
                                1.0,0.0,0.0,1.0,             //H
                                0.1,0.0,0.0,0.1,             //Q
                                1.0,0.0,0.0,1.0,             //R
                                0.0,0.0,                     //X
                                0.0);                        //Ut
  // Kalman_2_init(&Gz_Kalman,     1.0,t,0.0,1.0,               //F
  //                               0.5*t*t,t,                   //B
  //                               1.0,0.0,0.0,1.0,             //H
  //                               0.1,0.0,0.0,0.1,             //Q
  //                               0.1,0.0,0.0,0.1,             //R
  //                               0.0,0.0,                     //X
  //                               0.0);                        //Ut
  imu_timer_create();
  vTaskDelay(10000);
  uint32_t time_k = xTaskGetTickCount();
  imu_intr_enable_and_start();
  for(;;)
  {
    xSemaphoreTake(Sensor_get_data,portMAX_DELAY);
    imu_timer_stop();
    ICM_42688P_read_ACC_GYRO(&ICM_Ax,&ICM_Ay,&ICM_Az,&ICM_Gx,&ICM_Gy,&ICM_Gz);
    BMI270_read_ACC_GYRO(&BMI_Ax,&BMI_Ay,&BMI_Az,&BMI_Gx,&BMI_Gy,&BMI_Gz);
    t = 0.001 * (float)(xTaskGetTickCount() - time_k);
    time_k = xTaskGetTickCount();
    Yaw_angle += (ICM_Gz - ICM_Gz_offset) * t;
    Yaw_angle_BMI += (BMI_Gz - BMI_Gz_offset) * t;
    Kalman_2_cal(&Gz_Kalman_ICM,1.0,t,0.0,1.0,                      //F
                            0.5*t*t,t,                          //B
                            0.0,                                //Ut
                            Yaw_angle,(ICM_Gz - ICM_Gz_offset));        //Z_1,Z_2
    Kalman_2_cal(&Gz_Kalman_BMI,1.0,t,0.0,1.0,                      //F
                                0.5*t*t,t,                          //B
                                0.0,                                //Ut
                                Yaw_angle_BMI,(BMI_Gz - BMI_Gz_offset));        //Z_1,Z_2
    Gz = (Gz_Kalman_BMI.X[1] + Gz_Kalman_ICM.X[1]) / 2.0f;
    Yaw += Gz * t;
    // Kalman_2_cal(&Gz_Kalman,1.0,t,0.0,1.0,                      //F
    //                             0.5*t*t,t,                          //B
    //                             0.0,                                //Ut
    //                             Yaw,Gz);        //Z_1,Z_2
    ICM_temp = ICM_42688P_read_Temp();
    BMI_temp = BMI270_read_Temp();
    imu_timer_restart();
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
  indicator_set(80,0,0);
  vTaskDelay(250);
  indicator_breath_init(&indicator_Bre,0,0,255,255);
  for(;;)
  {
    indicator_breath_cal(&indicator_Bre);
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

#define WIFI_LINE_NUM   10
/**
 * @brief 
 * 
 * @param arg 
 */
void Task_UpMonitor(void *arg)
{
  union
  {
    float data[WIFI_LINE_NUM + 1];
    char out[WIFI_LINE_NUM * 4 + 4];
  }Line;
  myWifi_init();
  myWifi_start();
  vTaskDelay(5000);
  my_wifi_vofa_init();
  vTaskDelay(1000);
  Line.out[WIFI_LINE_NUM * 4] = 0x00;
  Line.out[WIFI_LINE_NUM * 4 + 1] = 0x00;
  Line.out[WIFI_LINE_NUM * 4 + 2] = 0x80;
  Line.out[WIFI_LINE_NUM * 4 + 3] = 0x7F;
  for(;;)
  {
    // Line.data[0] = Gz_Kalman.X[0];
    // Line.data[0] = ICM_temp;
    // // Line.data[1] = Gy;
    // Line.data[1] = Yaw_angle;
    // Line.data[2] = ICM_Gz;
    // Line.data[3] = Gz_Kalman.X[1];
    // Line.data[4] = Yaw_Kalman_angle;

    // Line.data[0] = Gz_Kalman_BMI.X[0];
    // Line.data[1] = Gz_Kalman.X[0];
    // Line.data[2] = Gz_Kalman.X[1];
    // Line.data[3] = BMI_Gz;
    // Line.data[4] = Gz_Kalman_BMI.X[1];

    Line.data[0] = Gz;
    Line.data[1] = Yaw;
    Line.data[2] = ICM_Gz;
    Line.data[3] = BMI_Gx;
    Line.data[4] = Gz_Kalman_ICM.X[0];
    Line.data[5] = Gz_Kalman_BMI.X[0];
    Line.data[6] = Gz_Kalman_ICM.X[1];
    Line.data[7] = Gz_Kalman_BMI.X[1];
    Line.data[8] = ICM_temp;
    Line.data[9] = BMI_temp;

    myWifi_vofa_send(Line.out,WIFI_LINE_NUM * 4 + 4);
    vTaskDelay(1);
  }
}

//intr

/**
 * @brief 
 * 
 * @param arg 
 */
void Task_Wifi_Recv(void *arg)
{
  vTaskDelay(20 * 1000);
  static char* wifi_rec_buff;
  for(;;)
  {
    wifi_rec_buff = myWifi_vofa_recv();
    if(*wifi_rec_buff == 'U')
    {
      //close other wifi
      vTaskSuspend(UpMonitor_Handle);
      myWifi_vofa_stop();

      ESP_LOGI("OTA","Now going to OTA....");
      indicator_breath_set(&indicator_Bre,0,255,0,255);
      OTA_update();
    }
    vTaskDelay(10);
  }
}
