#ifndef BSP_IIC__
#define BSP_IIC__

#include "driver/i2c.h"

void iic_master_init(uint8_t iic_port,uint8_t SDA_IO,uint8_t SCL_IO,uint32_t IIC_frq);
void iic_slave_init(uint8_t iic_port,uint8_t SDA_IO,uint8_t SCL_IO,uint16_t SLAVE_ADDR,uint32_t MAXSPEED,uint32_t rx_buf_len,uint32_t tx_buf_len);
void iic_send(uint8_t iic_port,uint8_t dst_addr,uint8_t *iic_data,uint32_t data_len);
void iic_read(uint8_t iic_port,uint8_t dst_addr,uint8_t *iic_data,uint32_t data_len);

#endif
