#include <stdio.h>
#include "sensor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

#define RX_BUFF_MAX_LEN             20
#define TX_BUFF_MAX_LEN             20

#define ICM_42688P_GYRO_FS          2000.0    //dps
#define ICM_42688P_ACC_FS           4.0       //g

static const char* TAG = "sensor:";

uint8_t Rx_Data_42688p[RX_BUFF_MAX_LEN] = {0};
uint8_t Rx_Data_BMI270[RX_BUFF_MAX_LEN] = {0};
uint8_t Rx_Data_BMP388[RX_BUFF_MAX_LEN] = {0};
uint8_t Tx_Data_42688p[TX_BUFF_MAX_LEN] = {0};
uint8_t Tx_Data_BMI270[TX_BUFF_MAX_LEN] = {0};
uint8_t Tx_Data_BMP388[TX_BUFF_MAX_LEN] = {0};

//spi device handle
spi_device_handle_t icm_42688p;
spi_device_handle_t bmi_270;
spi_device_handle_t bmp_388;

void sensor_init(void)
{
  ESP_LOGI(TAG,"Init");
  ICM_42688P_init();
  BMI270_init();
  BMP388_init();
}

void ICM_42688P_init(void)
{
  spi_reg_device_to_bus(6,&icm_42688p);

  // //Reset icm-42688P
  // Tx_Data_42688p[0] = DEVICE_CONFIG | ADDR_WRITE;
  // spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  // memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  // vTaskDelay(100);
  // //Check connect
  // Tx_Data_42688p[0] = WHO_AM_I | ADDR_READ;
  // spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  // memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  // if(Rx_Data_42688p[1] == 0x47)
  //   ESP_LOGI(TAG,"ICM-42688P connected");
  // else
  //   ESP_LOGI(TAG,"ICM-42688P unconnect......");
  // //Check reset
  // Tx_Data_42688p[0] = INT_STATUS | ADDR_READ;
  // spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  // memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  // if(Rx_Data_42688p[1] == 0x10)
  //   ESP_LOGI(TAG,"ICM-42688P reset success");
  // else
  //   ESP_LOGI(TAG,"ICM-42688P reset fail......");
  // //Set Scale And ODR
  // Tx_Data_42688p[0] = GYRO_CONFIG0 | ADDR_WRITE;
  // Tx_Data_42688p[1] = 0x01;
  // spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  // memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  // Tx_Data_42688p[0] = ACCEL_CONFIG0 | ADDR_WRITE;
  // Tx_Data_42688p[1] = 0x41;
  // spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  // memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  // //Disable FIFO
  // Tx_Data_42688p[0] = FIFO_CONFIG | ADDR_WRITE;
  // Tx_Data_42688p[1] = 0x00;
  // spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  // memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  // //Disable FSYNC
  // Tx_Data_42688p[0] = FSYNC_CONFIG | ADDR_WRITE;
  // Tx_Data_42688p[1] = 0x00;
  // spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  // memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  // Tx_Data_42688p[0] = TMST_CONFIG | ADDR_WRITE;
  // Tx_Data_42688p[1] = 0x21;
  // spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  // memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  // //Set default timestamp resolution 16us (Mobile use cases)
  // Tx_Data_42688p[0] = TMST_CONFIG | ADDR_WRITE;
  // Tx_Data_42688p[1] = 0x29;
  // spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  // memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  // //FIFO cfg
  // Tx_Data_42688p[0] = INTF_CONFIG0 | ADDR_WRITE;
  // Tx_Data_42688p[1] = 0x50;
  // spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  // memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  // Tx_Data_42688p[0] = FIFO_CONFIG | ADDR_WRITE;
  // Tx_Data_42688p[1] = 0x80;
  // spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  // memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  // Tx_Data_42688p[0] = TMST_CONFIG | ADDR_WRITE;
  // Tx_Data_42688p[1] = 0x29;
  // spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  // memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  // Tx_Data_42688p[0] = FIFO_CONFIG1 | ADDR_WRITE;
  // Tx_Data_42688p[1] = 0x2C;
  // spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  // memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  // Tx_Data_42688p[0] = FIFO_CONFIG2 | ADDR_WRITE;
  // Tx_Data_42688p[1] = 0x01;
  // spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  // memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  // Tx_Data_42688p[0] = INT_SOURCE0 | ADDR_WRITE;
  // Tx_Data_42688p[1] = 0x10;
  // spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  // memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  // //Enable push pull on INT1 to avoid moving in Test Mode after a soft reset
  // Tx_Data_42688p[0] = INT_CONFIG | ADDR_WRITE;
  // Tx_Data_42688p[1] = 0x1B;
  // spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  // memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  // //Set interrupt config
  // Tx_Data_42688p[0] = INT_SOURCE0 | ADDR_WRITE;
  // Tx_Data_42688p[1] = 0x14;
  // spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  // memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  // Tx_Data_42688p[0] = REG_BANK_SEL | ADDR_WRITE;
  // Tx_Data_42688p[1] = 0x04;       //switch into bank 4
  // spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  // memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  // Tx_Data_42688p[0] = INT_SOURCE8 | ADDR_WRITE;
  // Tx_Data_42688p[1] = 0x08;
  // spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  // memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  // Tx_Data_42688p[0] = REG_BANK_SEL | ADDR_WRITE;
  // Tx_Data_42688p[1] = 0x00;       //switch into bank 0
  // spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  // memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  // /* Set the ASY_RESET_DISABLE bit to 0 (async enabled) in order to chop Tpulse as soon as interrupt status is read
	//  * Guideline is to set the ASY_RESET_DISABLE bit to 0 in pulse mode
	//  * No effect in latch mode */
  // Tx_Data_42688p[0] = INT_CONFIG1 | ADDR_WRITE;
  // Tx_Data_42688p[1] = 0x00;
  // spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  // memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  // /* Set the UI filter order to 2 for both gyro and accel */
  // Tx_Data_42688p[0] = GYRO_CONFIG1 | ADDR_WRITE;
  // Tx_Data_42688p[1] = 0x16;
  // spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  // memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  // Tx_Data_42688p[0] = ACCEL_CONFIG1 | ADDR_WRITE;
  // Tx_Data_42688p[1] = 0x0D;
  // spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  // memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  // /* FIFO packets are 16bit format by default (i.e. high res is disabled) */
  // /* For retro-compatibility, configure WOM to compare current sample with the previous sample and to produce signal when all axis exceed 52 mg */
  // Tx_Data_42688p[0] = REG_BANK_SEL | ADDR_WRITE;
  // Tx_Data_42688p[1] = 0x04;       //switch into bank 4
  // spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  // memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  // Tx_Data_42688p[0] = ACCEL_WOM_X_THR | ADDR_WRITE;
  // Tx_Data_42688p[1] = 0x13;
  // Tx_Data_42688p[2] = 0x13;
  // Tx_Data_42688p[3] = 0x13;
  // spi_connect_start(CS_42688P,&icm_42688p,4 * 8,Tx_Data_42688p,Rx_Data_42688p);
  // memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  // Tx_Data_42688p[0] = REG_BANK_SEL | ADDR_WRITE;
  // Tx_Data_42688p[1] = 0x00;       //switch into bank 0
  // spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  // memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  // Tx_Data_42688p[0] = SMD_CONFIG | ADDR_WRITE;
  // Tx_Data_42688p[1] = 0x0C;
  // spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  // memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  // //Set Scale And ODR
  // Tx_Data_42688p[0] = GYRO_CONFIG0 | ADDR_WRITE;
  // Tx_Data_42688p[1] = 0x06;
  // spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  // memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  // Tx_Data_42688p[0] = ACCEL_CONFIG0 | ADDR_WRITE;
  // Tx_Data_42688p[1] = 0x46;
  // spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  // memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  // //Run
  // Tx_Data_42688p[0] = PWR_MGMT0 | ADDR_WRITE;
  // Tx_Data_42688p[1] = 0x0F;
  // spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  // memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  // vTaskDelay(100);      //Wait
  // //Read WHO AM I
  // Tx_Data_42688p[0] = WHO_AM_I | ADDR_READ;
  // spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  // memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  // if(Rx_Data_42688p[1] == 0x47)
  //   ESP_LOGI(TAG,"ICM-42688P Set OK");
  // else
  //   ESP_LOGI(TAG,"ICM-42688P Set Fail......");

  
  //reset icm-42688p
  Tx_Data_42688p[0] = DEVICE_CONFIG | ADDR_WRITE;
  Tx_Data_42688p[1] = 0x01;
  spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  vTaskDelay(100);
  Tx_Data_42688p[0] = WHO_AM_I | ADDR_READ;
  spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  if(Rx_Data_42688p[1] == 0x47)
    ESP_LOGI(TAG,"ICM-42688P connected");
  else
    ESP_LOGI(TAG,"ICM-42688P unconnect......");
  //FSYNC Set
  // Tx_Data_42688p[0] = FSYNC_CONFIG | ADDR_WRITE;
  //INT Pin Set
  Tx_Data_42688p[0] = INT_CONFIG | ADDR_WRITE;
  Tx_Data_42688p[1] = 0x1B;
  spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  //INT Set
  Tx_Data_42688p[0] = INT_CONFIG1 | ADDR_WRITE;
  Tx_Data_42688p[1] = 0x60;
  spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  
  //INTF Set
  Tx_Data_42688p[0] = INTF_CONFIG0 | ADDR_WRITE;
  Tx_Data_42688p[1] = 0xB0;
  spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  //FIFO Stop on Full
  Tx_Data_42688p[0] = FIFO_CONFIG | ADDR_WRITE;
  Tx_Data_42688p[1] = 0x80;
  spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  //Time Stamp Set
  Tx_Data_42688p[0] = TMST_CONFIG | ADDR_WRITE;
  Tx_Data_42688p[1] = 0x27;
  spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  //FIFO Set
  Tx_Data_42688p[0] = FIFO_CONFIG1 | ADDR_READ;
  Tx_Data_42688p[1] = 0x2F;         // FIFO cout ???
  spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  Tx_Data_42688p[0] = FIFO_CONFIG2 | ADDR_READ;
  Tx_Data_42688p[1] = 0x01;
  spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  
  //INTF Set
  Tx_Data_42688p[0] = INTF_CONFIG1 | ADDR_WRITE;
  Tx_Data_42688p[1] = 0x98;
  spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);

  //Open FIFO
  Tx_Data_42688p[0] = FIFO_CONFIG | ADDR_WRITE;
  Tx_Data_42688p[1] = 0x40;
  spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  //change  into BANK 1
  Tx_Data_42688p[0] = REG_BANK_SEL | ADDR_WRITE;
  Tx_Data_42688p[1] = 0x01;
  spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  //Filter Set
  Tx_Data_42688p[0] = GYRO_CONFIG_STATIC2 | ADDR_WRITE;
  Tx_Data_42688p[1] = 0x03;         //close filter
  spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  //change  into BANK 0
  Tx_Data_42688p[0] = REG_BANK_SEL | ADDR_WRITE;
  Tx_Data_42688p[1] = 0x00;
  spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  //Run
  Tx_Data_42688p[0] = PWR_MGMT0 | ADDR_WRITE;
  Tx_Data_42688p[1] = 0x0F;
  spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  vTaskDelay(100);      //Wait
  //Set Scale And ODR
  Tx_Data_42688p[0] = GYRO_CONFIG0 | ADDR_WRITE;
  // Tx_Data_42688p[1] = 0x01;     //32K
  Tx_Data_42688p[1] = 0x02;     //16K
  spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  Tx_Data_42688p[0] = ACCEL_CONFIG0 | ADDR_WRITE;
  // Tx_Data_42688p[1] = 0x41;     //32K
  Tx_Data_42688p[1] = 0x42;     //16K
  spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  //Read WHO AM I
  Tx_Data_42688p[0] = WHO_AM_I | ADDR_READ;
  spi_connect_start(CS_42688P,&icm_42688p,2 * 8,Tx_Data_42688p,Rx_Data_42688p);
  memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  if(Rx_Data_42688p[1] == 0x47)
    ESP_LOGI(TAG,"ICM-42688P Set OK");
  else
    ESP_LOGI(TAG,"ICM-42688P Set Fail......");
  //Self test
  
}

float ICM_42688P_read_Temp(void)
{
  float temp = 0.0;
  Tx_Data_42688p[0] = TEMP_DATA1 | ADDR_READ;
  spi_connect_start(CS_42688P,&icm_42688p,3 * 8,Tx_Data_42688p,Rx_Data_42688p);
  memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  // ESP_LOGI(TAG,"ICM-42688P Temp: High bit  %hX",Rx_Data_42688p[1]);
  // ESP_LOGI(TAG,"ICM-42688P Temp: LOW bit  %hX",Rx_Data_42688p[2]);
  temp = ( (float)(int16_t)( (Rx_Data_42688p[1] << 8 ) + Rx_Data_42688p[2] ) / 132.48 ) + 25.0;
  ESP_LOGI(TAG,"ICM-42688P Temp: %f",temp);

  return temp;
}

uint16_t ICM_42688P_read_Temp_u16(void)
{
  Tx_Data_42688p[0] = TEMP_DATA1 | ADDR_READ;
  spi_connect_start(CS_42688P,&icm_42688p,3 * 8,Tx_Data_42688p,Rx_Data_42688p);
  memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);

  return (Rx_Data_42688p[1] << 8 ) + Rx_Data_42688p[2];
}

void ICM_42688P_read_ACC(float *Ax,float *Ay,float *Az)
{
  Tx_Data_42688p[0] = ACCEL_DATA_X1 | ADDR_READ;
  spi_connect_start(CS_42688P,&icm_42688p,7 * 8,Tx_Data_42688p,Rx_Data_42688p);
  memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  *Ax = ( (float)(int16_t)( (Rx_Data_42688p[1] << 8) + Rx_Data_42688p[2] ) * ICM_42688P_ACC_FS / 32768.0);
  *Ay = ( (float)(int16_t)( (Rx_Data_42688p[3] << 8) + Rx_Data_42688p[4] ) * ICM_42688P_ACC_FS / 32768.0);
  *Az = ( (float)(int16_t)( (Rx_Data_42688p[5] << 8) + Rx_Data_42688p[6] ) * ICM_42688P_ACC_FS / 32768.0);
}

void ICM_42688P_read_GYRO(float *Gx,float *Gy,float *Gz)
{
  Tx_Data_42688p[0] = GYRO_DATA_X1 | ADDR_READ;
  spi_connect_start(CS_42688P,&icm_42688p,7 * 8,Tx_Data_42688p,Rx_Data_42688p);
  memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  *Gx = ( (float)(int16_t)( (Rx_Data_42688p[1] << 8) + Rx_Data_42688p[2] ) * ICM_42688P_GYRO_FS / 32768.0);
  *Gy = ( (float)(int16_t)( (Rx_Data_42688p[3] << 8) + Rx_Data_42688p[4] ) * ICM_42688P_GYRO_FS / 32768.0);
  *Gz = ( (float)(int16_t)( (Rx_Data_42688p[5] << 8) + Rx_Data_42688p[6] ) * ICM_42688P_GYRO_FS / 32768.0);
}

void ICM_42688P_read_ACC_GYRO(float *Ax,float *Ay,float *Az,float *Gx,float *Gy,float *Gz)
{
  Tx_Data_42688p[0] = ACCEL_DATA_X1 | ADDR_READ;
  spi_connect_start(CS_42688P,&icm_42688p,13 * 8,Tx_Data_42688p,Rx_Data_42688p);
  memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  *Ax = ( (float)(int16_t)( (Rx_Data_42688p[1] << 8) + Rx_Data_42688p[2] ) * ICM_42688P_ACC_FS / 32768.0);
  *Ay = ( (float)(int16_t)( (Rx_Data_42688p[3] << 8) + Rx_Data_42688p[4] ) * ICM_42688P_ACC_FS / 32768.0);
  *Az = ( (float)(int16_t)( (Rx_Data_42688p[5] << 8) + Rx_Data_42688p[6] ) * ICM_42688P_ACC_FS / 32768.0);
  *Gx = ( (float)(int16_t)( (Rx_Data_42688p[7] << 8) + Rx_Data_42688p[8] ) * ICM_42688P_GYRO_FS / 32768.0);
  *Gy = ( (float)(int16_t)( (Rx_Data_42688p[9] << 8) + Rx_Data_42688p[10] ) * ICM_42688P_GYRO_FS / 32768.0);
  *Gz = ( (float)(int16_t)( (Rx_Data_42688p[11] << 8) + Rx_Data_42688p[12] ) * ICM_42688P_GYRO_FS / 32768.0);
}

void ICM_42688P_read_FIFO(void)
{
  uint16_t Data_Len;
  Tx_Data_42688p[0] = FIFO_COUNTH | ADDR_READ;
  spi_connect_start(CS_42688P,&icm_42688p,3 * 8,Tx_Data_42688p,Rx_Data_42688p);
  memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
  Data_Len = (Rx_Data_42688p[1] << 8 ) + Rx_Data_42688p[2];
  if(Data_Len > 0)
  {
    Tx_Data_42688p[0] = FIFO_DATA_ICM | ADDR_READ;
    spi_connect_start(CS_42688P,&icm_42688p,(Data_Len +1) * 8,Tx_Data_42688p,Rx_Data_42688p);
    memset(Tx_Data_42688p,0,TX_BUFF_MAX_LEN);
    for(uint8_t i = 0;i < Data_Len;i++)
    {
      ESP_LOGI(TAG,"ICM-42688P FIFO: %hX",Rx_Data_42688p[i+1]);
    }
  }
  else
    ESP_LOGI(TAG,"ICM-42688P FIFO: Empty.....");
}

void BMI270_init(void)
{
  spi_reg_device_to_bus(6,&bmi_270);
  Tx_Data_BMI270[0] = CHIP_ID | ADDR_READ;
  // Tx_Data_BMI270[1] = 0x00;
  spi_connect_start(CS_BMI270,&bmi_270,4 * 8,Tx_Data_BMI270,Rx_Data_BMI270);
  memset(Tx_Data_BMI270,0,TX_BUFF_MAX_LEN);
  if(Rx_Data_BMI270[3] == 0x24)
    ESP_LOGI(TAG,"BMI-270 connected");
  else
    ESP_LOGI(TAG,"BMI-270 unconnect......");
}

void BMP388_init(void)
{
  spi_reg_device_to_bus(6,&bmp_388);
  Tx_Data_BMP388[0] = CHIP_ID | ADDR_READ;
  // Tx_Data_BMP388[1] = 0x00;
  spi_connect_start(CS_BMP388,&bmp_388,3 * 8,Tx_Data_BMP388,Rx_Data_BMP388);
  memset(Tx_Data_BMP388,0,TX_BUFF_MAX_LEN);
  if(Rx_Data_BMP388[2] == 0x50)
    ESP_LOGI(TAG,"BMP-388 connected");
  else
    ESP_LOGI(TAG,"BMP-388 unconnect......");
}

void imu_10_data_1_out_to_uart(float Ax,float Ay,float Az,float Gx,float Gy,float Gz,uint32_t index,float Out[10][6])
{
  Out[index][0] = Ax;
  Out[index][1] = Ay;
  Out[index][2] = Az;
  Out[index][3] = Gx;
  Out[index][4] = Gy;
  Out[index][5] = Gz;
}
