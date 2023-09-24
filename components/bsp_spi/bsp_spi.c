#include <stdio.h>
#include "bsp_spi.h"

/**
 * @brief 
 * 
 * @param mosi_pin 
 * @param miso_pin 
 * @param spi_clk 
 */
void spi_bus_init(uint8_t mosi_pin,uint8_t miso_pin,uint8_t spi_clk)
{
  spi_bus_config_t bus_conf = {
    .mosi_io_num = mosi_pin,
    .miso_io_num = miso_pin,
    .sclk_io_num = spi_clk,
    .quadhd_io_num = -1,
    .quadwp_io_num = -1,
    .max_transfer_sz = 4092,
  };
  //init spi bus
  spi_bus_initialize(SPI_HOST,&bus_conf,SPI_DMA_CH_AUTO);
  //CS pull up
  gpio_pad_select_gpio(CS_42688P);
  gpio_set_direction(CS_42688P, GPIO_MODE_OUTPUT);
  gpio_set_level(CS_42688P, 1);

  gpio_pad_select_gpio(CS_BMI270);      //BMI270 switch into SPI Mode
  gpio_set_direction(CS_BMI270, GPIO_MODE_OUTPUT);
  gpio_set_level(CS_BMI270, 0);
  gpio_set_level(CS_BMI270, 1);

  gpio_pad_select_gpio(CS_BMP388);
  gpio_set_direction(CS_BMP388, GPIO_MODE_OUTPUT);
  gpio_set_level(CS_BMP388, 1);

  gpio_pad_select_gpio(CS_BOX);
  gpio_set_direction(CS_BOX, GPIO_MODE_OUTPUT);
  gpio_set_level(CS_BOX, 1);
}

/**
 * @brief 
 * 
 * @param cs_io 
 * @param dev_handle 
 */
void spi_reg_device_to_bus(uint8_t queue_size,spi_device_handle_t *dev_handle)
{
  spi_device_interface_config_t device_conf = {
    .clock_speed_hz = 10 * 1000000,
    .mode = 0,
    .spics_io_num = -1,
    .queue_size = queue_size,
    .command_bits = 0,
    .address_bits = 0,
    .input_delay_ns = 0,
    .dummy_bits = 0,
  };

  //register device connect to device
  spi_bus_add_device(SPI_HOST,&device_conf,dev_handle);
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
void spi_connect_start(uint8_t cs_io,spi_device_handle_t *dev_handle,uint32_t len,void *txbuffer,void * rxbuffer)
{
  spi_transaction_t t = {0};
  t.length = len;
  t.tx_buffer = txbuffer;
  t.rx_buffer = rxbuffer;
  gpio_set_level(cs_io, 0);
  spi_device_polling_transmit(*dev_handle,&t);
  // ESP_ERROR_CHECK(spi_device_polling_transmit(icm_42688p,&t));
  gpio_set_level(cs_io, 1);
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
