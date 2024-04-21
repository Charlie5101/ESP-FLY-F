#include <stdio.h>
#include "bsp_iic.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"

//Mutex Lock Handle
SemaphoreHandle_t IIC_Mutex_Lock;

/**
 * @brief 
 * 
 * @param iic_port 
 * @param SDA_IO 
 * @param SCL_IO 
 * @param IIC_frq 
 */
void iic_master_init(uint8_t iic_port,uint8_t SDA_IO,uint8_t SCL_IO,uint32_t IIC_frq)
{
  //Mutex Lock
  IIC_Mutex_Lock = xSemaphoreCreateMutex();
  if(IIC_Mutex_Lock == NULL)
  {
    ESP_LOGE("IIC","Mutex Lock Create Fail......");
  }
  xSemaphoreTake(IIC_Mutex_Lock,portMAX_DELAY);     //Lock

  //config iic
  uint8_t iic_master_port = iic_port;
  i2c_config_t conf = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = SDA_IO,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = SCL_IO,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .master.clk_speed = IIC_frq,
    .clk_flags = 0,
  };
  i2c_param_config(iic_master_port,&conf);
  //install iic
  i2c_driver_install(iic_master_port,conf.mode,0,0,0);
  xSemaphoreGive(IIC_Mutex_Lock);     //UNLock
}

/**
 * @brief 
 * 
 * @param iic_port 
 * @param SDA_IO 
 * @param SCL_IO 
 * @param SLAVE_ADDR 
 * @param MAXSPEED 
 * @param rx_buf_len 
 * @param tx_buf_len 
 */
void iic_slave_init(uint8_t iic_port,uint8_t SDA_IO,uint8_t SCL_IO,uint16_t SLAVE_ADDR,uint32_t MAXSPEED,uint32_t rx_buf_len,uint32_t tx_buf_len)
{
  //config iic
  uint8_t iic_slave_port = iic_port;
  i2c_config_t conf_slave = {
    .sda_io_num = SDA_IO,
    .sda_pullup_en = GPIO_PULLUP_ENABLE,
    .scl_io_num = SCL_IO,
    .scl_pullup_en = GPIO_PULLUP_ENABLE,
    .mode = I2C_MODE_SLAVE,
    .slave.addr_10bit_en = 0,
    .slave.slave_addr = SLAVE_ADDR,
    .slave.maximum_speed = MAXSPEED,
    .clk_flags = 0,
  };
  i2c_param_config(iic_slave_port,&conf_slave);
  //install iic
  i2c_driver_install(iic_slave_port,conf_slave.mode,rx_buf_len,tx_buf_len,0);
}

/**
 * @brief 
 * 
 * @param iic_port 
 * @param dst_addr 
 * @param iic_data 
 * @param data_len 
 */
void iic_send(uint8_t iic_port,uint8_t dst_addr,uint8_t *iic_data,uint32_t data_len)
{
  xSemaphoreTake(IIC_Mutex_Lock,portMAX_DELAY);     //Lock
  //create one cmd link
  i2c_cmd_handle_t iic_cmd_link = i2c_cmd_link_create();
  i2c_master_start(iic_cmd_link);
  i2c_master_write_byte(iic_cmd_link,(dst_addr << 1)|I2C_MASTER_WRITE,1);
  i2c_master_write(iic_cmd_link,iic_data,data_len,1);
  i2c_master_stop(iic_cmd_link);
  i2c_master_cmd_begin(iic_port,iic_cmd_link,0xFFFF);
  i2c_cmd_link_delete(iic_cmd_link);
  xSemaphoreGive(IIC_Mutex_Lock);     //UNLock
}

/**
 * @brief 
 * 
 * @param iic_port 
 * @param dst_addr 
 * @param iic_data 
 * @param data_len 
 */
void iic_read(uint8_t iic_port,uint8_t dst_addr,uint8_t *iic_data,uint32_t data_len)
{
  xSemaphoreTake(IIC_Mutex_Lock,portMAX_DELAY);     //Lock
  //create one cmd link
  i2c_cmd_handle_t iic_cmd_link = i2c_cmd_link_create();
  i2c_master_start(iic_cmd_link);
  i2c_master_write_byte(iic_cmd_link,(dst_addr << 1)|I2C_MASTER_READ,1);
  i2c_master_read(iic_cmd_link,iic_data,data_len,1);
  i2c_master_stop(iic_cmd_link);
  i2c_master_cmd_begin(iic_port,iic_cmd_link,0xFFFF);
  i2c_cmd_link_delete(iic_cmd_link);
  xSemaphoreGive(IIC_Mutex_Lock);     //UNLock
}
