#ifndef SENSOR__
#define SENSOR__

#include "bsp_spi.h"

/*ICM-42688p Register Map*/
//BANK 0
#define DEVICE_CONFIG         0x11
#define DRIVE_CONFIG          0x13
#define INT_CONFIG            0x14
#define FIFO_CONFIG           0x16
#define TEMP_DATA1            0x1D
#define TEMP_DATA0            0x1E
#define ACCEL_DATA_X1         0x1F
#define ACCEL_DATA_X0         0x20
#define ACCEL_DATA_Y1         0x21
#define ACCEL_DATA_Y0         0x22
#define ACCEL_DATA_Z1         0x23
#define ACCEL_DATA_Z0         0x24
#define GYRO_DATA_X1          0x25
#define GYRO_DATA_X0          0x26
#define GYRO_DATA_Y1          0x27
#define GYRO_DATA_Y0          0x28
#define GYRO_DATA_Z1          0x29
#define GYRO_DATA_Z0          0x2A
#define TMST_FSYNCH           0x2B
#define TMST_FSYNCL           0x2C
#define INT_STATUS            0x2D
#define FIFO_COUNTH           0x2E
#define FIFO_COUNTL           0x2F
#define FIFO_DATA_ICM         0x30            //redefine
#define APEX_DATA0            0x31
#define APEX_DATR1            0x32
#define APEX_DATA2            0x33
#define APEX_DATR3            0x34
#define APEX_DATA4            0x35
#define APEX_DATR5            0x36
#define INT_STATUS2           0x37
#define INT_STATUS3           0x38
#define SIGNAL_PATH_RESET     0x4B
#define INTF_CONFIG0          0x4C
#define INTF_CONFIG1          0x4D
#define PWR_MGMT0             0x4E
#define GYRO_CONFIG0          0x4F
#define ACCEL_CONFIG0         0x50
#define GYRO_CONFIG1          0x51
#define GYRO_ACCEL_CONFIG0    0x52
#define ACCEL_CONFIG1         0x53
#define TMST_CONFIG           0x54
#define APEX_CONFIG0          0x56
#define SMD_CONFIG            0x57
#define FIFO_CONFIG1          0x5F
#define FIFO_CONFIG2          0x60
#define FIFO_CONFIG3          0x61
#define FSYNC_CONFIG          0x62
#define INT_CONFIG0           0x63
#define INT_CONFIG1           0X64
#define INT_SOURCE0           0X65
#define INT_SOURCE1           0X66
#define INT_SOURCE3           0X68
#define INT_SOURCE4           0X68
#define FIFO_LOST_PKT0        0X6C
#define FIFO_LOST_PKT1        0X6D
#define SELF_TEST_CONFIG      0x70
#define WHO_AM_I              0x75
#define REG_BANK_SEL          0x76
//BANK 1
#define SENSOR_CONFIG0        0x03
#define GYRO_CONFIG_STATIC2   0x0B
#define GYRO_CONFIG_STATIC3   0x0C
#define GYRO_CONFIG_STATIC4   0x0D
#define GYRO_CONFIG_STATIC5   0x0E
#define GYRO_CONFIG_STATIC6   0x0F
#define GYRO_CONFIG_STATIC7   0x10
#define GYRO_CONFIG_STATIC8   0x11
#define GYRO_CONFIG_STATIC9   0x12
#define GYRO_CONFIG_STATIC10  0x13
#define XG_ST_DATA            0x5F
#define YG_ST_DATA            0x60
#define ZG_ST_DATA            0x61
#define TMSTVAL0              0x62
#define TMSTVAL1              0x63
#define TMSTVAL2              0x64
#define INTF_CONFIG4          0x7A
#define INTF_CONFIG5          0x7B
#define INTF_CONFIG6          0x7C
//BANK 2
#define ACCEL_CONFIG_STATIC2  0x03
#define ACCEL_CONFIG_STATIC3  0x04
#define ACCEL_CONFIG_STATIC4  0x05
#define XA_ST_DATA            0x3B
#define YA_ST_DATA            0x3C
#define ZA_ST_DATA            0x3D
//BANK 4
#define APEX_CONFIG1          0x40
#define APEX_CONFIG2          0x41
#define APEX_CONFIG3          0x42
#define APEX_CONFIG4          0x43
#define APEX_CONFIG5          0x44
#define APEX_CONFIG6          0x45
#define APEX_CONFIG7          0x46
#define APEX_CONFIG8          0x47
#define APEX_CONFIG9          0x48
#define ACCEL_WOM_X_THR       0x4A
#define ACCEL_WOM_Y_THR       0x4B
#define ACCEL_WOM_Z_THR       0x4C
#define INT_SOURCE6           0x4D
#define INT_SOURCE7           0x4E
#define INT_SOURCE8           0x4F
#define INT_SOURCE9           0x50
#define INT_SOURCE10          0x51
#define OFFSET_USER0          0x77
#define OFFSET_USER1          0x78
#define OFFSET_USER2          0x79
#define OFFSET_USER3          0x7A
#define OFFSET_USER4          0x7B
#define OFFSET_USER5          0x7C
#define OFFSET_USER6          0x7D
#define OFFSET_USER7          0x7E
#define OFFSET_USER8          0x7F

/*BMI270 Register Map*/
#define CMD                   0x7E
#define PWR_CTRL              0x7D
#define PWR_CONF              0x7C
#define OFFSET_6              0x77
#define OFFSET_5              0x76
#define OFFSET_4              0x75
#define OFFSET_3              0x74
#define OFFSET_2              0x73
#define OFFSET_1              0x72
#define OFFSET_0              0x71
#define NV_CONF               0x70
#define GYR_SELF_TEST_AXES    0x6E
#define ACC_SELF_TEST         0x6D
#define DRV                   0x6C
#define IF_CONF               0x6B
#define NVM_CONF              0x6A
#define GYR_CRT_CONF          0x69
#define AUX_IF_TRIM           0x68
#define INTERNAL_ERROR        0x5F
#define INIT_DATA             0x5E
#define INIT_ADDR_1           0x5C
#define INIT_ADDR_0           0x5B
#define INIT_CTRL             0x59
#define INT_MAP_DATA          0x58
#define INT2_MAP_FETA         0x57
#define INT1_MAP_FEAT         0x56
#define INT_LATCH             0x55
#define INT2_IO_CTRL          0x54
#define INT1_IO_CTRL          0x53
#define ERR_REG_MSK           0x52
#define AUX_WR_DATA           0x4F
#define AUX_WR_ADDR           0x4E
#define AUX_RD_ADDR           0x4D
#define AUX_IF_CONF           0x4C
#define AUX_DEV_ID            0x4B
#define SATURATION            0x4A
#define FIFO_CONFG_1          0x49
#define FIFO_CONFG_0          0x48
#define FIFO_WTM_1            0x47
#define FIFO_WTM_0            0x46
#define FIFO_DOWNS            0x45
#define AUX_CONF              0x44
#define GYR_RANGE             0x43
#define GYR_CONF              0x42
#define ACC_RANGE             0x41
#define ACC_CONF              0x40
#define FEATURES_15           0x3F
#define FEATURES_14           0x3E
#define FEATURES_13           0x3D
#define FEATURES_12           0x3C
#define FEATURES_11           0x3B
#define FEATURES_10           0x3A
#define FEATURES_9            0x39
#define FEATURES_8            0x38
#define FEATURES_7            0x37
#define FEATURES_6            0x36
#define FEATURES_5            0x35
#define FEATURES_4            0x34
#define FEATURES_3            0x33
#define FEATURES_2            0x32
#define FEATURES_1            0x31
#define FEATURES_0            0x30
#define FEAT_PAGE             0x2F
#define FIFO_DATA_BMI         0x26  //redefine
#define FIFO_LENGTH_1         0x25
#define FIFO_LENGTH_0         0x24
#define TEMPERATURE_1         0x23
#define TEMPERATURE_0         0x22
#define INTERNAL_STATUS       0x21
#define WR_GEST_ACT           0x20
#define SC_OUT_1              0x1F
#define SC_OUT_0              0x1E
#define INT_STATUS_1          0x1D
#define INT_STATUS_0          0x1C
#define EVENT                 0x1B
#define SENSORTIME_2          0x1A
#define SENSORTIME_1          0x19
#define SENSORTIME_0          0x18
#define DATA_19               0x17
#define DATA_18               0x16
#define DATA_17               0x15
#define DATA_16               0x14
#define DATA_15               0x13
#define DATA_14               0x12
#define DATA_13               0x11
#define DATA_12               0x10
#define DATA_11               0x0F
#define DATA_10               0x0E
#define DATA_9                0x0D
#define DATA_8                0x0C
#define DATA_7                0x0B
#define DATA_6                0x0A
#define DATA_5                0x09
#define DATA_4                0x08
#define DATA_3                0x07
#define DATA_2                0x06
#define DATA_1                0x05
#define DATA_0                0x04
#define STATUS                0x03
#define ERR_REG               0x02
#define CHIP_ID               0x00

/*BMP388 Register Map*/

void sensor_init(void);
void ICM_42688P_init(void);
float ICM_42688P_read_Temp(void);
uint16_t ICM_42688P_read_Temp_u16(void);
void ICM_42688P_read_ACC(float *Ax,float *Ay,float *Az);
void ICM_42688P_read_GYRO(float *Gx,float *Gy,float *Gz);
void ICM_42688P_read_ACC_GYRO(float *Ax,float *Ay,float *Az,float *Gx,float *Gy,float *Gz);
void ICM_42688P_read_FIFO(void);
void ICM_42688P_Get_Bais(float* Gx_B,float* Gy_B,float* Gz_B);
void BMI270_init(void);
float BMI270_read_Temp(void);
void BMI270_read_GYRO(float *Gx,float *Gy,float *Gz);
void BMI270_read_ACC_GYRO(float *Ax,float *Ay,float *Az,float *Gx,float *Gy,float *Gz);
void BMI270_Get_Bais(float* Gx_B,float* Gy_B,float* Gz_B);
void BMP388_init(void);

#endif
