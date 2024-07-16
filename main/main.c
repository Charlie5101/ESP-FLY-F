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
#include "PID.h"
#include "receiver.h"

/*task config*/
#define Task_MAIN_Stack       4096
#define Task_MAIN_Prio        4
#define Task_sensor_Stack     4096 * 8
#define Task_sensor_Prio      4
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
#define Task_Link_Check_Stack 4096
#define Task_Link_Check_Prio  3
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
TaskHandle_t Link_Check_Handle;
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
void Task_Link_Check(void *arg);
void Task_UpMonitor(void *arg);
void Task_Wifi_Recv(void *arg);

/*Class*/
Senser_Classdef Senser;
DRAM_ATTR myPID_Classdef Roll_pid;
DRAM_ATTR myPID_Classdef Pitch_pid;
DRAM_ATTR myPID_Classdef Yaw_pid;
Receiver_Classdef Receiver;
DSHOT_Classdef DSHOT;
Servo_Classdef Servo;
Indicator_Classdef Indicator;

/*Var*/
DRAM_ATTR float pitch_target = 0.0f;
DRAM_ATTR float roll_target = 0.0f;
DRAM_ATTR float yaw_target = 0.0f;

SemaphoreHandle_t Motor_Adjust;

float ICM_temp = 0.0f;
float BMI_temp = 0.0f;
float ICM_Ax,ICM_Ay,ICM_Az,ICM_Gx,ICM_Gy,ICM_Gz;
float BMI_Ax,BMI_Ay,BMI_Az,BMI_Gx,BMI_Gy,BMI_Gz;
float t = 0.0001;

float ICM_Roll = 0.0;
float ICM_Pitch = 0.0;
float ICM_Yaw = 0.0;

float Gx = 0.0;
float Gy = 0.0;
float Gz = 0.0;
float Roll = 0.0;
float Pitch = 0.0;
float Yaw = 0.0;
float ARoll = 0.0;
float APitch = 0.0;

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
  spi_bus_init(SPI_HOST,SPI_MOSI,SPI_MISO,SPI_SCLK,4092 * 3);
  spi_bus_init(SPI_SECOND_HOST,indicator_io,-1,-1,4092);
  //Create semaphores
  Motor_Adjust = xSemaphoreCreateBinary();
  //Create tasks
  xTaskCreatePinnedToCore(Task_MAIN,      "main task",      Task_MAIN_Stack,      NULL, Task_MAIN_Prio,       &MAIN_Handle,      1);
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
  myPID_Class_init(&Roll_pid, 1.0, 0.0, 0.0, 1000.0, 2000.0, 100.0, 3000.0);
  myPID_Class_init(&Pitch_pid, 1.0, 0.0, 0.0, 1000.0, 2000.0, 100.0, 3000.0);
  myPID_Class_init(&Yaw_pid, 1.0, 0.0, 0.0, 1000.0, 2000.0, 100.0, 3000.0);
  control_timer_create();
  vTaskDelay(1000);
  control_intr_enable_and_start();
  for(;;)
  {
    xSemaphoreTake(Pid_Contrl,portMAX_DELAY);
    Roll_pid.update(&Roll_pid, roll_target, Roll);
    Pitch_pid.update(&Pitch_pid, pitch_target, Pitch);
    Yaw_pid.update(&Yaw_pid, yaw_target, Yaw);
    Roll_pid.cal(&Roll_pid);
    Pitch_pid.cal(&Pitch_pid);
    Yaw_pid.cal(&Yaw_pid);
    xSemaphoreGive(Motor_Adjust);
  }
}

/**
 * @brief 
 * 
 * @param arg
 */
void Task_sensor(void *arg)
{
  Senser_Class_init(&Senser);
  imu_timer_create();
  vTaskDelay(10000);
  uint32_t time_k = xTaskGetTickCount();
  imu_intr_enable_and_start();

  for(;;)
  {
    xSemaphoreTake(Sensor_get_data, portMAX_DELAY);
    imu_timer_stop();
    
    t = 0.001 * (float)(xTaskGetTickCount() - time_k);
    time_k = xTaskGetTickCount();

    Senser.Kalman.update(&Senser.Kalman, t, 0.0f, 0.0f, 0.0f);

    Gx = Senser.Kalman.dRoll;
    Roll = Senser.Kalman.Roll;
    Gy = Senser.Kalman.dPitch;
    Pitch = Senser.Kalman.Pitch;
    Gz = Senser.Kalman.dYaw;
    Yaw = Senser.Kalman.Yaw;
    ARoll = Senser.Kalman.ARoll;
    APitch = Senser.Kalman.APitch;
    
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
  Indicator_Class_init(&Indicator);
  vTaskDelay(10);
  Indicator.Color_Set(&Indicator, 80, 0, 0);
  vTaskDelay(250);
  Indicator.Bre_init(&Indicator.Bre, 0, 0, 255, 255);
  Indicator.Send_Message(&Indicator, 0, 0, 255, Breath, 255);
  for(;;)
  {
    Indicator.Adjust(&Indicator);
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
  static uint16_t M1_throttle = 0;

  DSHOT_Class_init(&DSHOT);
  Servo_Class_init(&Servo);

  Servo.Set_Out(1, 23.0);
  Servo.Set_Out(2, 35.0);
  Servo.Set_Out(3, 67.0);
  Servo.Set_Out(4, 89.0);
  for(uint8_t i=1;i<5;i++)
  {
    DSHOT.ESC_unLock(&DSHOT, i);
  }
  vTaskDelay(10000);
  
  for(;;)
  {
    xSemaphoreTake(Motor_Adjust, portMAX_DELAY);
    // ESP_LOGI("MOTOR", "%d:", M1_throttle);
    for(uint8_t i=1;i<5;i++)
    {
      DSHOT.Set_Throttle(&DSHOT, i, M1_throttle);
    }
    // vTaskDelay(100);
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
  Receiver_Class_init(&Receiver);
  Receiver.crsf.crc8_init(&Receiver);
  xTaskCreatePinnedToCore(Task_Link_Check,  "Receiver Link Check",  Task_Link_Check_Stack, NULL, Task_Link_Check_Prio,  &Link_Check_Handle, 0);
  for(;;)
  {
    xSemaphoreTake(Uart2_data_rec, portMAX_DELAY);
    if( Receiver.crsf.crc_check(&Receiver, &Receiver.dtmp[2]) == true )
    {
      Receiver.LinkNum = 100;
      Receiver.crsf.decode(&Receiver);
    }
  }
}

/**
 * @brief 
 * 
 * @param arg 
 */
void Task_Link_Check(void *arg)
{
  for(;;)
  {
    if(Receiver.LinkNum > 0)
    {
      if(Receiver.LinkState != true)
        Indicator.Send_Message(&Indicator, 0, 0, 255, Breath, 255);
      Receiver.LinkState = true;
      Receiver.LinkNum--;
    }
    else
    {
      if(Receiver.LinkState != false)
        Indicator.Send_Message(&Indicator, 255, 0, 0, Breath, 60);
      Receiver.LinkState = false;
      // ESP_LOGW("LINK STATE","FALSE");
    }
    vTaskDelay(3);
  }
}

#define WIFI_LINE_NUM   11
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
  vTaskDelay(1000);
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

    // Line.data[0] = (Receiver.main_data.ch2 * 1024 + 1024);
    // // Line.data[0] = Receiver.main_data.ch0 * 1000.0f;
    // Line.data[1] = Receiver.main_data.ch1 * 1000.0f;
    // Line.data[2] = Receiver.main_data.ch2 * 1000.0f;
    // Line.data[3] = Receiver.main_data.ch3 * 1000.0f;

    Line.data[0] = Gx;
    Line.data[1] = Roll;
    Line.data[2] = Senser.Kalman.Gx;
    Line.data[3] = Gy;
    Line.data[4] = Pitch;
    Line.data[5] = Senser.Kalman.Gy;
    Line.data[6] = Gz;
    Line.data[7] = Yaw;
    Line.data[8] = Senser.Kalman.Gz;
    Line.data[9] = ARoll;
    Line.data[10] = APitch;

    myWifi_vofa_send(Line.out,WIFI_LINE_NUM * 4 + 4);
    Socket_Service();
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
      // Indicator.Bre_Color_Set(&Indicator, 0, 255, 0, 255);
      Indicator.Send_Message(&Indicator, 255, 0, 255, Breath, 255);
      OTA_update();
    }
    vTaskDelay(10);
  }
}

void uart_event_task(QueueHandle_t* queue)
{
  uart_event_t event;
  uint8_t *dtmp = (uint8_t *)malloc(UART_RD_BUFF_SIZE);
  for(;;)
  {
    // Waiting for UART event.
    if (xQueueReceive(*queue, (void *)&event, (TickType_t)portMAX_DELAY))
    {
      bzero(dtmp, UART_RD_BUFF_SIZE);
      // ESP_LOGI("UART", "uart[%d] event:", EX_UART_NUM);
      switch (event.type)
      {
      // Event of UART receving data
      /*We'd better handler data event fast, there would be much more data events than
      other types of events. If we take too much time on data event, the queue might
      be full.*/
      case UART_DATA:
        // ESP_LOGI("UART", "[UART DATA]: %d", event.size);
        uart_read_bytes(EX_UART_NUM, dtmp, event.size, portMAX_DELAY);
        // ESP_LOGI("UART", "[DATA EVT]:");
        // uart_write_bytes(EX_UART_NUM, (const char *)dtmp, event.size);

        Receiver.rec_data(&Receiver, dtmp, event.size);
        xSemaphoreGive(Uart2_data_rec);

        break;
      // Event of HW FIFO overflow detected
      case UART_FIFO_OVF:
        ESP_LOGI("UART", "hw fifo overflow");
        // If fifo overflow happened, you should consider adding flow control for your application.
        // The ISR has already reset the rx FIFO,
        // As an example, we directly flush the rx buffer here in order to read more data.
        uart_flush_input(EX_UART_NUM);
        xQueueReset(*queue);
        break;
      // Event of UART ring buffer full
      case UART_BUFFER_FULL:
        ESP_LOGI("UART", "ring buffer full");
        // If buffer full happened, you should consider increasing your buffer size
        // As an example, we directly flush the rx buffer here in order to read more data.
        uart_flush_input(EX_UART_NUM);
        xQueueReset(*queue);
        break;
      // Event of UART RX break detected
      case UART_BREAK:
        ESP_LOGI("UART", "uart rx break");
        break;
      // Event of UART parity check error
      case UART_PARITY_ERR:
        ESP_LOGI("UART", "uart parity error");
        break;
      // Event of UART frame error
      case UART_FRAME_ERR:
        ESP_LOGI("UART", "uart frame error");
        break;
      // Others
      default:
        ESP_LOGI("UART", "uart event type: %d", event.type);
        break;
      }
    }
  }
}
