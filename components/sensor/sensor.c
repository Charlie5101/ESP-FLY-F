#include <stdio.h>
#include "sensor.h"
#include "esp_log.h"
#include "freertos/task.h"

static const char* TAG = "sensor:";

uint8_t Rx_Data_42688p[10] = {0};
uint8_t Rx_Data_BMI270[10] = {0};
uint8_t Rx_Data_BMP388[10] = {0};
uint8_t Tx_Data_42688p[10] = {0};
uint8_t Tx_Data_BMI270[10] = {0};
uint8_t Tx_Data_BMP388[10] = {0};

//spi device handle
spi_device_handle_t icm_42688p;
spi_device_handle_t bmi_270;
spi_device_handle_t bmp_388;

void sensor_init(void)
{
  ESP_LOGI(TAG,"Init");

  spi_reg_device_to_bus(6,&icm_42688p);
  Tx_Data_42688p[0] = DEVICE_CONFIG | ADDR_WRITE;
  Tx_Data_42688p[1] = 0x01;
  spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);   //reset icm-42688p
  vTaskDelay(100);
  Tx_Data_42688p[0] = WHO_AM_I | ADDR_READ;
  Tx_Data_42688p[1] = 0x00;
  spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  if(Rx_Data_42688p[1] == 0x47)
    ESP_LOGI(TAG,"ICM-42688P connected");
  else
    ESP_LOGI(TAG,"ICM-42688P unconnect......");

  spi_reg_device_to_bus(6,&bmi_270);
  Tx_Data_BMI270[0] = CHIP_ID | ADDR_READ;
  Tx_Data_BMI270[1] = 0x00;
  spi_connect_start(CS_BMI270,&bmi_270,4 * 8,Tx_Data_BMI270,Rx_Data_BMI270);
  if(Rx_Data_BMI270[3] == 0x24)
    ESP_LOGI(TAG,"BMI-270 connected");
  else
    ESP_LOGI(TAG,"BMI-270 unconnect......");

  spi_reg_device_to_bus(6,&bmp_388);
  Tx_Data_BMP388[0] = CHIP_ID | ADDR_READ;
  Tx_Data_BMP388[1] = 0x00;
  spi_connect_start(CS_BMP388,&bmp_388,3 * 8,Tx_Data_BMP388,Rx_Data_BMP388);
  if(Rx_Data_BMP388[2] == 0x50)
    ESP_LOGI(TAG,"BMP-388 connected");
  else
    ESP_LOGI(TAG,"BMP-388 unconnect......");

}
