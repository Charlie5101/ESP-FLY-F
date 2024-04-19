#include <stdio.h>
#include "black_box.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BLACK_BOX_SPEED_HZ          40 * 1000000  //Hz

static const char* TAG = "black box:";

uint8_t Tx_Data_BOX[10] = {0};
uint8_t Rx_Data_BOX[10] = {0};

//spi device handle
spi_device_handle_t black_box;

void black_box_init()
{
  //CS pull up
  gpio_set_direction(CS_BOX, GPIO_MODE_OUTPUT);
  gpio_set_level(CS_BOX, 1);

  spi_reg_device_to_bus(SPI_HOST,6,&black_box,BLACK_BOX_SPEED_HZ,BOX_SPI_MODE);
  Tx_Data_BOX[0] = 0x9F | ADDR_WRITE;
  Tx_Data_BOX[1] = 0x00;
  spi_connect_start(BOX_HOST,CS_BOX,&black_box,5 * 8,Tx_Data_BOX,Rx_Data_BOX);
  if(Rx_Data_BOX[2] == 0xEF && Rx_Data_BOX[3] == 0xAA && Rx_Data_BOX[4] == 0x21)
    ESP_LOGI(TAG,"BLACK BOX connected");
  else
    ESP_LOGI(TAG,"BLACK BOX unconnect......");
}
