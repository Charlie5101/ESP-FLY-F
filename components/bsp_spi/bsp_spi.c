#include <stdio.h>
#include "bsp_spi.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

//Mutex Lock Handle
SemaphoreHandle_t SPI_Mutex_Lock;
SemaphoreHandle_t SPI_SECOND_Mutex_Lock;

/**
 * @brief 
 * 
 * @param mosi_pin 
 * @param miso_pin 
 * @param spi_clk 
 */ 
void spi_bus_init(uint8_t spi_host,uint8_t mosi_pin,uint8_t miso_pin,uint8_t spi_clk,uint32_t max_tran_size)
{
  if(spi_host == SPI_HOST)
  {
    //Mutex Lock
    SPI_Mutex_Lock = xSemaphoreCreateMutex();
    if(SPI_Mutex_Lock == NULL)
    {
      ESP_LOGE("SPI","Mutex Lock Create Fail......");
    }
    xSemaphoreTake(SPI_Mutex_Lock,portMAX_DELAY);     //Lock

    spi_bus_config_t bus_conf = {
      .mosi_io_num = mosi_pin,
      .miso_io_num = miso_pin,
      .sclk_io_num = spi_clk,
      .quadhd_io_num = -1,
      .quadwp_io_num = -1,
      .max_transfer_sz = max_tran_size,
    };
    //init spi bus
    spi_bus_initialize(spi_host,&bus_conf,SPI_DMA_CH_AUTO);

    xSemaphoreGive(SPI_Mutex_Lock);     //UNLock
  }
  else if(spi_host == SPI_SECOND_HOST)
  {
    //Mutex Lock
    SPI_SECOND_Mutex_Lock = xSemaphoreCreateMutex();
    if(SPI_SECOND_Mutex_Lock == NULL)
    {
      ESP_LOGE("SPI","Mutex Lock Create Fail......");
    }
    xSemaphoreTake(SPI_SECOND_Mutex_Lock,portMAX_DELAY);     //Lock

    spi_bus_config_t bus_conf = {
      .mosi_io_num = mosi_pin,
      .miso_io_num = miso_pin,
      .sclk_io_num = spi_clk,
      .quadhd_io_num = -1,
      .quadwp_io_num = -1,
      .max_transfer_sz = max_tran_size,
    };
    //init spi bus
    spi_bus_initialize(spi_host,&bus_conf,SPI_DMA_CH_AUTO);

    xSemaphoreGive(SPI_SECOND_Mutex_Lock);     //UNLock
  }
}

/**
 * @brief 
 * 
 * @param cs_io 
 * @param dev_handle 
 */
void spi_reg_device_to_bus(uint8_t spi_host,uint8_t queue_size,spi_device_handle_t *dev_handle,uint32_t clock_speed_hz,uint8_t mode)
{
  if(spi_host == SPI_HOST)
  {
    xSemaphoreTake(SPI_Mutex_Lock,portMAX_DELAY);     //Lock

    spi_device_interface_config_t device_conf = {
      .clock_speed_hz = clock_speed_hz,
      .mode = mode,
      .spics_io_num = -1,
      .queue_size = queue_size,
      .command_bits = 0,
      .address_bits = 0,
      .input_delay_ns = 0,
      .dummy_bits = 0,
    };

    //register device connect to device
    spi_bus_add_device(spi_host,&device_conf,dev_handle);

    xSemaphoreGive(SPI_Mutex_Lock);     //UNLock
  }
  else if(spi_host == SPI_SECOND_HOST)
  {
    xSemaphoreTake(SPI_SECOND_Mutex_Lock,portMAX_DELAY);     //Lock

    spi_device_interface_config_t device_conf = {
      .clock_speed_hz = clock_speed_hz,
      .mode = 0,
      .spics_io_num = -1,
      .queue_size = queue_size,
      .command_bits = 0,
      .address_bits = 0,
      .input_delay_ns = 0,
      .dummy_bits = 0,
    };

    //register device connect to device
    spi_bus_add_device(spi_host,&device_conf,dev_handle);

    xSemaphoreGive(SPI_SECOND_Mutex_Lock);     //UNLock
  }
}

/**
 * @brief 
 * 
 * @param dev_handle 
 * @param spi_tx_buffer 
 * @param tx_data_len 
 * @param spi_rx_buffer 
 * @param rx_data_len 
 */
void spi_connect_start(uint8_t spi_host,uint8_t cs_io,spi_device_handle_t *dev_handle,uint32_t len,void *txbuffer,void * rxbuffer)
{
  if(spi_host == SPI_HOST)
  {
    xSemaphoreTake(SPI_Mutex_Lock,portMAX_DELAY);     //Lock
  }
  else if(spi_host == SPI_SECOND_HOST)
  {
    xSemaphoreTake(SPI_SECOND_Mutex_Lock,portMAX_DELAY);     //Lock
  }

  spi_transaction_t t = {0};
  t.length = len;
  t.tx_buffer = txbuffer;
  t.rx_buffer = rxbuffer;
  gpio_set_level(cs_io, 0);
  spi_device_polling_transmit(*dev_handle,&t);
  // ESP_ERROR_CHECK(spi_device_polling_transmit(icm_42688p,&t));
  gpio_set_level(cs_io, 1);

  if(spi_host == SPI_HOST)
  {
    xSemaphoreGive(SPI_Mutex_Lock);     //UNLock
  }
  else if(spi_host == SPI_SECOND_HOST)
  {
    xSemaphoreGive(SPI_SECOND_Mutex_Lock);     //UNLock
  }
  
}

/**
 * @brief 
 * 
 * @param dev_handle 
 */
void spi_connect_stop(spi_device_handle_t *dev_handle)
{
  spi_device_release_bus(*dev_handle);
}
