#ifndef BSP_SPI__
#define BSP_SPI__

#include "driver/spi_master.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"

#define SPI_HOST      SPI2_HOST

#define SPI_MOSI      11
#define SPI_MISO      13
#define SPI_SCLK      12
#define ADDR_READ     0x80
#define ADDR_WRITE    0x00

#define SPI_SECOND_HOST     SPI3_HOST

void spi_bus_init(uint8_t spi_host,uint8_t mosi_pin,uint8_t miso_pin,uint8_t spi_clk,uint32_t max_tran_size);
void spi_reg_device_to_bus(uint8_t spi_host,uint8_t queue_size,spi_device_handle_t *dev_handle,uint32_t clock_speed_hz,uint8_t mode);
void spi_connect_start(uint8_t spi_host,uint8_t cs_io,spi_device_handle_t *dev_handle,uint32_t len,void *txbuffer,void * rxbuffer);
void spi_connect_stop(spi_device_handle_t *dev_handle);

#endif
