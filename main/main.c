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
#include "Control.h"
#include "bat_adc.h"

/*task config*/
#define Task_MAIN_Stack       4096
#define Task_MAIN_Prio        18
#define Task_sensor_Stack     4096
#define Task_sensor_Prio      18
#define Task_black_box_Stack  4096
#define Task_black_box_Prio   3
#define Task_indicator_Stack  4096
#define Task_indicator_Prio   3
#define Task_buzzer_Stack     4096
#define Task_buzzer_Prio      3
#define Task_bat_adc_Stack    4096
#define Task_bat_adc_Prio     3
#define Task_motor_Stack      4096
#define Task_motor_Prio       6
#define Task_GPS_Stack        4096
#define Task_GPS_Prio         3
#define Task_RGB_LED_Stack    4096
#define Task_RGB_LED_Prio     3
#define Task_receiver_Stack   4096
#define Task_receiver_Prio    6
#define Task_Link_Check_Stack 4096
#define Task_Link_Check_Prio  4
#define Task_receiver_TLM_Stack 4096
#define Task_receiver_TLM_Prio  4
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
TaskHandle_t Receiver_TLM_Handle;
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
void Task_receiver_TLM(void *arg);
void Task_UpMonitor(void *arg);
void Task_Wifi_Recv(void *arg);

/*Class*/
DRAM_ATTR Senser_Classdef Senser;
DRAM_ATTR Control_Classdef Control;
Receiver_Classdef Receiver;
DSHOT_Classdef DSHOT;
Servo_Classdef Servo;
Indicator_Classdef Indicator;
BAT_Voltage_Classdef BAT;
My_Wifi_Classdef My_Wifi;

/*Var*/
DRAM_ATTR float pitch_target = 0.0f;
DRAM_ATTR float roll_target = 0.0f;
DRAM_ATTR float yaw_target = 0.0f;

SemaphoreHandle_t Motor_Adjust;

float t = 0.0001;

float Roll = 0.0;
float Pitch = 0.0;
float Yaw = 0.0;

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
  SemaphoreHandle_t *Init_Seq = (SemaphoreHandle_t *)pvPortMalloc(sizeof(SemaphoreHandle_t));
  *Init_Seq = xSemaphoreCreateBinary();
  xSemaphoreGive(*Init_Seq);
  //Create tasks
  xSemaphoreTake(*Init_Seq, portMAX_DELAY);
  xTaskCreatePinnedToCore(Task_MAIN,      "main task",      Task_MAIN_Stack,      (void*)Init_Seq, Task_MAIN_Prio,       &MAIN_Handle,      1);
  ESP_LOGW("SYS", "main task created");
  #if INDICATOR_ENABLE == 1
  xSemaphoreTake(*Init_Seq, portMAX_DELAY);
  xTaskCreatePinnedToCore(Task_indicator, "indicator LED",  Task_indicator_Stack, (void*)Init_Seq, Task_indicator_Prio,  &Indicator_Handle, 0);
  ESP_LOGW("SYS", "indicator LED task created");
  #endif
  #if RGB_LED_ENABLE == 1
  xSemaphoreTake(*Init_Seq, portMAX_DELAY);
  xTaskCreatePinnedToCore(Task_RGB_LED,   "RGB LED out",    Task_RGB_LED_Stack,   (void*)Init_Seq, Task_RGB_LED_Prio,    &RGB_LED_Handle,   0);
  ESP_LOGW("SYS", "RGB LED out task created");
  #endif
  #if SENSOR_ENABLE == 1
  xSemaphoreTake(*Init_Seq, portMAX_DELAY);
  xTaskCreatePinnedToCore(Task_sensor,    "IMU task",       Task_sensor_Stack,    (void*)Init_Seq, Task_sensor_Prio,     &Sensor_Handle,    1);
  ESP_LOGW("SYS", "IMU task created");
  #endif
  #if BLACK_BOX_ENABLE == 1
  xSemaphoreTake(*Init_Seq, portMAX_DELAY);
  xTaskCreatePinnedToCore(Task_black_box, "flash black box",Task_black_box_Stack, (void*)Init_Seq, Task_black_box_Prio,  &Black_box_Handle, 0);
  ESP_LOGW("SYS", "flash black box task created");
  #endif
  #if BAT_ADC_ENABLE == 1
  xSemaphoreTake(*Init_Seq, portMAX_DELAY);
  xTaskCreatePinnedToCore(Task_bat_adc,   "Battery voltage",Task_bat_adc_Stack,   (void*)Init_Seq, Task_bat_adc_Prio,    &Bat_adc_Handle,   0);
  ESP_LOGW("SYS", "Battery voltage task created");
  #endif
  #if RECEIVER_ENABLE == 1
  xSemaphoreTake(*Init_Seq, portMAX_DELAY);
  xTaskCreatePinnedToCore(Task_receiver,   "Receiver",      Task_receiver_Stack,  (void*)Init_Seq, Task_receiver_Prio,   &Receiver_Handle,  0);
  ESP_LOGW("SYS", "Receiver task created");
  #endif
  #if MOTOR_ENABLE == 1
  xSemaphoreTake(*Init_Seq, portMAX_DELAY);
  xTaskCreatePinnedToCore(Task_motor,     "motor out",      Task_motor_Stack,     (void*)Init_Seq, Task_motor_Prio,      &Motor_Handle,     0);
  ESP_LOGW("SYS", "motor out task created");
  #endif
  #if UPMONITOR_ENABLE == 1
  xSemaphoreTake(*Init_Seq, portMAX_DELAY);
  xTaskCreatePinnedToCore(Task_UpMonitor,  "UpMonitor",     Task_UpMonitor_Stack, (void*)Init_Seq, Task_UpMonitor_Prio,  &UpMonitor_Handle, 0);
  ESP_LOGW("SYS", "UpMonitor task created");
  #endif
  #if WIFI_RECV_ENABLE == 1
  xSemaphoreTake(*Init_Seq, portMAX_DELAY);
  xTaskCreatePinnedToCore(Task_Wifi_Recv,  "Wifi Receive",  Task_Wifi_Recv_Stack, (void*)Init_Seq, Task_Wifi_Recv_Prio,  &Wifi_Recv_Handle, 0);
  ESP_LOGW("SYS", "Wifi Receive task created");
  #endif
  #if GPS_ENABLE == 1
  xSemaphoreTake(*Init_Seq, portMAX_DELAY);
  xTaskCreatePinnedToCore(Task_GPS,       "GPS/BDS",        Task_GPS_Stack,       (void*)Init_Seq, Task_GPS_Prio,        &GPS_Handle,       0);
  ESP_LOGW("SYS", "GPS/BDS task created");
  #endif
  #if BUZZER_ENABLE == 1
  xSemaphoreTake(*Init_Seq, portMAX_DELAY);
  xTaskCreatePinnedToCore(Task_buzzer,    "buzzer",         Task_buzzer_Stack,    (void*)Init_Seq, Task_buzzer_Prio,     &Buzzer_Handle,    0);
  ESP_LOGW("SYS", "buzzer task created");
  #endif
  xSemaphoreTake(*Init_Seq, portMAX_DELAY);
  vPortFree(*Init_Seq);
  ESP_LOGW("SYS", "All task created");
  return 0;
}

/**
 * @brief 
 * 
 * @param arg 
 */
void Task_MAIN(void *arg)
{
  Control_PID_param* PID_param = (Control_PID_param*)pvPortMalloc(84);
  PID_param->Roll.Kp = 200.0;
  PID_param->Roll.Ki = 0.0;
  PID_param->Roll.Kd = 0.0;
  PID_param->Roll.P_Limit = 10000.0;
  PID_param->Roll.I_Limit = 2000.0;
  PID_param->Roll.D_Limit = 100.0;
  PID_param->Roll.OUT_Limit = 10000.0;

  PID_param->Pitch.Kp = 200.0;
  PID_param->Pitch.Ki = 0.0;
  PID_param->Pitch.Kd = 0.0;
  PID_param->Pitch.P_Limit = 10000.0;
  PID_param->Pitch.I_Limit = 2000.0;
  PID_param->Pitch.D_Limit = 100.0;
  PID_param->Pitch.OUT_Limit = 10000.0;

  PID_param->Yaw.Kp = 200.0;
  PID_param->Yaw.Ki = 0.0;
  PID_param->Yaw.Kd = 0.0;
  PID_param->Yaw.P_Limit = 10000.0;
  PID_param->Yaw.I_Limit = 2000.0;
  PID_param->Yaw.D_Limit = 100.0;
  PID_param->Yaw.OUT_Limit = 10000.0;

  Control_Class_init(&Control, *PID_param);
  vPortFree(PID_param);
  control_timer_create();
  vTaskDelay(1000);
  control_intr_enable_and_start();
  xSemaphoreGive(*((SemaphoreHandle_t*)arg));
  for(;;)
  {
    xSemaphoreTake(Pid_Contrl,portMAX_DELAY);
    roll_target = 45.0 * Receiver.main_data.ch0;
    pitch_target = 45.0 * Receiver.main_data.ch1;
    yaw_target = -30.0 * Receiver.main_data.ch3;
    Control.update(&Control, Receiver.main_data.ch2, roll_target, Roll, pitch_target, Pitch, yaw_target, Yaw);
    Control.cal(&Control);
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
  static uint8_t BMP_COUNT = 0;
  Senser_Class_init(&Senser);
  imu_timer_create();
  vTaskDelay(1000);
  uint32_t time_k = xTaskGetTickCount();
  imu_intr_enable_and_start();
  xSemaphoreGive(*((SemaphoreHandle_t*)arg));
  for(;;)
  {
    xSemaphoreTake(Sensor_get_data, portMAX_DELAY);
    imu_timer_stop();
    
    t = 0.001 * (float)(xTaskGetTickCount() - time_k);
    time_k = xTaskGetTickCount();

    Senser.ICM42688P.read_ACC_GYRO(&Senser.ICM42688P, &Senser.ICM42688P.Row_data.Ax, &Senser.ICM42688P.Row_data.Ay, &Senser.ICM42688P.Row_data.Az,
                                                      &Senser.ICM42688P.Row_data.Gx, &Senser.ICM42688P.Row_data.Gy, &Senser.ICM42688P.Row_data.Gz);
    Senser.BMI270.read_ACC_GYRO(&Senser.BMI270, &Senser.BMI270.Row_data.Ax, &Senser.BMI270.Row_data.Ay, &Senser.BMI270.Row_data.Az,
                                                &Senser.BMI270.Row_data.Gx, &Senser.BMI270.Row_data.Gy, &Senser.BMI270.Row_data.Gz);

    Senser.Mahony_ICM.update(&Senser.Mahony_ICM, Senser.ICM42688P.Row_data.Gx, Senser.ICM42688P.Row_data.Gy, Senser.ICM42688P.Row_data.Gz,
                                                 Senser.ICM42688P.Row_data.Ax, Senser.ICM42688P.Row_data.Ay, Senser.ICM42688P.Row_data.Az, t);
    Senser.Mahony_BMI.update(&Senser.Mahony_BMI, Senser.BMI270.Row_data.Gx, Senser.BMI270.Row_data.Gy, Senser.BMI270.Row_data.Gz,
                                                 Senser.BMI270.Row_data.Ax, Senser.BMI270.Row_data.Ay, Senser.BMI270.Row_data.Az, t);

    Senser.Mahony_ICM.toEuler(&Senser.Mahony_ICM);
    Senser.Mahony_BMI.toEuler(&Senser.Mahony_BMI);
    Senser.Kalman.update(&Senser.Kalman,
                         Senser.Mahony_ICM.quaternion.q0, Senser.Mahony_ICM.quaternion.q1, Senser.Mahony_ICM.quaternion.q2, Senser.Mahony_ICM.quaternion.q3,
                         Senser.Mahony_BMI.quaternion.q0, Senser.Mahony_BMI.quaternion.q1, Senser.Mahony_BMI.quaternion.q2, Senser.Mahony_BMI.quaternion.q3);
    Senser.Kalman.toEuler(&Senser.Kalman);

    Senser.Mahony_ICM.quaternion.q0 = Senser.Mahony_BMI.quaternion.q0 = Senser.Kalman.OUT.q0;
    Senser.Mahony_ICM.quaternion.q1 = Senser.Mahony_BMI.quaternion.q1 = Senser.Kalman.OUT.q1;
    Senser.Mahony_ICM.quaternion.q2 = Senser.Mahony_BMI.quaternion.q2 = Senser.Kalman.OUT.q2;
    Senser.Mahony_ICM.quaternion.q3 = Senser.Mahony_BMI.quaternion.q3 = Senser.Kalman.OUT.q3;

    Roll = Senser.Mahony_ICM.Euler.Roll / 3.1415926f * 180.0f;
    Pitch = Senser.Mahony_ICM.Euler.Pitch / 3.1415926f * 180.0f;
    // Yaw = Senser.Mahony_ICM.Euler.Yaw / 3.1415926f * 180.0f;
    Yaw = Senser.Mahony_ICM.Euler.Total_Yaw / 3.1415926f * 180.0f;

    if(BMP_COUNT <= 200)
    {
      Senser.BMP388.ask_and_read(&Senser.BMP388);
      BMP_COUNT++;
    }
    else
    {
      BMP_COUNT = 0;
    }
    
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
  xSemaphoreGive(*((SemaphoreHandle_t*)arg));
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
  xSemaphoreGive(*((SemaphoreHandle_t*)arg));
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
void Task_RGB_LED(void *arg)
{
  xSemaphoreGive(*((SemaphoreHandle_t*)arg));
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
void Task_buzzer(void *arg)
{
  xSemaphoreGive(*((SemaphoreHandle_t*)arg));
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
  BAT_Voltage_Class_init(&BAT);
  // vTaskDelay(5000);
  xSemaphoreGive(*((SemaphoreHandle_t*)arg));
  for(;;)
  {
    BAT.read(&BAT);
    vTaskDelay(500);
  }
}

/**
 * @brief 
 * 
 * @param arg 
 */
void Task_motor(void *arg)
{
  DSHOT_Class_init(&DSHOT);
  Servo_Class_init(&Servo);

  Servo.Set_Out(1, 0.0);
  Servo.Set_Out(2, 0.0);
  Servo.Set_Out(3, 0.0);
  Servo.Set_Out(4, 0.0);
  for(uint8_t i=1;i<5;i++)
  {
    DSHOT.ESC_unLock(&DSHOT, i);
  }
  vTaskDelay(5000);
  xSemaphoreGive(*((SemaphoreHandle_t*)arg));
  for(;;)
  {
    xSemaphoreTake(Motor_Adjust, portMAX_DELAY);
    // ESP_LOGI("MOTOR", "%d:", M1_throttle);
    // ESP_LOGI("MOTOR", "%f:", Receiver.main_data.ch2 *2000);
    // ESP_LOGI("MOTOR", "%d:", Control.power_out.throttle_A);
    if(Receiver.main_data.ch4 > 0 && Receiver.LinkState == true)
    {
      DSHOT.Set_All_Throttle(&DSHOT, Control.power_out.throttle_A, Control.power_out.throttle_B, Control.power_out.throttle_C, Control.power_out.throttle_D);
    }
    else
    {
      DSHOT.Set_All_Throttle(&DSHOT, 0, 0, 0, 0);
    }
  }
}

/**
 * @brief 
 * 
 * @param arg 
 */
void Task_GPS(void *arg)
{
  xSemaphoreGive(*((SemaphoreHandle_t*)arg));
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
  xTaskCreatePinnedToCore(Task_receiver_TLM,  "Receiver TLM",  Task_receiver_TLM_Stack, NULL, Task_receiver_Prio,  &Receiver_TLM_Handle, 0);
  xSemaphoreGive(*((SemaphoreHandle_t*)arg));
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
  TickType_t xLastWakeTime = xTaskGetTickCount();
  for(;;)
  {
    if(Receiver.LinkNum > 0)
    {
      if(Receiver.LinkState != true)
      {
        Indicator.Send_Message(&Indicator, 0, 0, 255, Breath, 255);
        ESP_LOGI("REC", "Link Connected" );
      }
      Receiver.LinkState = true;
      Receiver.LinkNum--;
    }
    else
    {
      if(Receiver.LinkState != false)
      {
        Indicator.Send_Message(&Indicator, 255, 0, 0, Breath, 60);
        ESP_LOGI("REC", "Link Disconnected");
      }
      Receiver.LinkState = false;
      // ESP_LOGW("LINK STATE","FALSE");
    }
    // vTaskDelay(3);
    xTaskDelayUntil(&xLastWakeTime, 3);
  }
}

/**
 * @brief 
 * 
 * @param arg 
 */
void Task_receiver_TLM(void *arg)
{
  for(;;)
  {
    Receiver.crsf.bat_TLM_send(&Receiver, BAT.fvoltage, 0.0f, 1100, (uint8_t)((BAT.fvoltage - 14) / (17.4 - 14) * 100));
    Receiver.crsf.attitude_TLM_send(&Receiver, Senser.Kalman.Euler.Roll, Senser.Kalman.Euler.Pitch, Senser.Kalman.Euler.Yaw);
    vTaskDelay(100);
  }
}

#define WIFI_LINE_NUM   16
/**
 * @brief 
 * 
 * @param arg 
 */
void Task_UpMonitor(void *arg)
{
  // TickType_t xLastWakeTime = xTaskGetTickCount();
  union
  {
    float data[WIFI_LINE_NUM + 1];
    char out[WIFI_LINE_NUM * 4 + 4];
  }Line;
  My_Wifi_Class_init(&My_Wifi);
  My_Wifi.start();
  vTaskDelay(1000);
  // My_Wifi.vofa.TCP_init(&My_Wifi);
  My_Wifi.vofa.UDP_init(&My_Wifi);
  vTaskDelay(1000);
  Line.out[WIFI_LINE_NUM * 4] = 0x00;
  Line.out[WIFI_LINE_NUM * 4 + 1] = 0x00;
  Line.out[WIFI_LINE_NUM * 4 + 2] = 0x80;
  Line.out[WIFI_LINE_NUM * 4 + 3] = 0x7F;
  xSemaphoreGive(*((SemaphoreHandle_t*)arg));
  for(;;)
  {
    // Line.data[0] = Roll;
    // Line.data[1] = Pitch;
    // Line.data[2] = Yaw;
    // Line.data[3] = Control.power_out.throttle_A;
    // Line.data[4] = Control.power_out.throttle_B;
    // Line.data[5] = Control.power_out.throttle_C;
    // Line.data[6] = Control.power_out.throttle_D;
    // Line.data[7] = Control.Normal_Data.Roll;
    // Line.data[8] = Control.Normal_Data.Pitch;
    // Line.data[9] = Control.Normal_Data.Yaw;
    // Line.data[10] = Gx;
    // Line.data[11] = Gy;
    // Line.data[12] = Gz;

    //ACC Calibrations info
    // Line.data[0] = Senser.ICM42688P.Row_data.Ax;
    // Line.data[1] = Senser.ICM42688P.Row_data.Ay;
    // Line.data[2] = Senser.ICM42688P.Row_data.Az;
    // Line.data[3] = Senser.BMI270.Row_data.Ax;
    // Line.data[4] = Senser.BMI270.Row_data.Ay;
    // Line.data[5] = Senser.BMI270.Row_data.Az;

    Line.data[0] = Roll;
    Line.data[1] = Pitch;
    Line.data[2] = Yaw;
    // Line.data[3] = Senser.ICM42688P.Row_data.Gx;
    Line.data[3] = Control.distribute_var.motor_bit;
    Line.data[4] = Senser.ICM42688P.Row_data.Gy;
    Line.data[5] = Senser.ICM42688P.Row_data.Gz;
    // Line.data[6] = roll_target;
    // Line.data[7] = pitch_target;
    Line.data[6] = Senser.Mahony_ICM.ACC_NOMAL;
    Line.data[7] = Senser.Mahony_BMI.ACC_NOMAL;
    Line.data[8] = yaw_target;
    // Line.data[9] = Control.PID.Roll.data.Target;
    // Line.data[10] = Control.PID.Roll.data.Current;
    // Line.data[11] = Control.PID.Roll.OUT;
    // Line.data[9] = Control.PID.Pitch.data.Target;
    // Line.data[10] = Control.PID.Pitch.data.Current;
    // Line.data[11] = Control.PID.Pitch.OUT;
    Line.data[9] = Control.PID.Yaw.data.Target;
    Line.data[10] = Control.PID.Yaw.data.Current;
    Line.data[11] = Control.PID.Yaw.OUT;
    Line.data[12] = Control.power_out.throttle_A;
    Line.data[13] = Control.power_out.throttle_B;
    Line.data[14] = Control.power_out.throttle_C;
    Line.data[15] = Control.power_out.throttle_D;
    // My_Wifi.vofa.TCP_send(&My_Wifi, Line.out, WIFI_LINE_NUM * 4 + 4);
    My_Wifi.vofa.UDP_send(&My_Wifi, Line.out, WIFI_LINE_NUM * 4 + 4);
    My_Wifi.Socket_Service(&My_Wifi);
    vTaskDelay(1);
    // xTaskDelayUntil(&xLastWakeTime, 1);
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
  // vTaskDelay(20 * 1000);
  static char* wifi_rec_buff;
  xSemaphoreGive(*((SemaphoreHandle_t*)arg));
  for(;;)
  {
    // wifi_rec_buff = My_Wifi.vofa.TCP_recv(&My_Wifi);
    wifi_rec_buff = My_Wifi.vofa.UDP_recv(&My_Wifi);
    if(*wifi_rec_buff == 'U')
    {
      ESP_LOGI("OTA","Decte U");
      //close other wifi
      vTaskSuspend(UpMonitor_Handle);
      // My_Wifi.vofa.TCP_stop(&My_Wifi);
      My_Wifi.vofa.UDP_stop(&My_Wifi);
      ESP_LOGI("OTA","Now going to OTA....");
      Indicator.Send_Message(&Indicator, 255, 0, 255, Breath, 255);
      My_Wifi.OTA(&My_Wifi);
    }
    vTaskDelay(10);
  }
}

//service

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
