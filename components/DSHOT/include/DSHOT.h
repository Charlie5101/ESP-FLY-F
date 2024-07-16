#ifndef __DSHOT__
#define __DSHOT__

// #include <stdint.h>
#include <stdbool.h>
#include "bsp_rmt.h"

#define DSHOT100        100 * 1000
#define DSHOT300        300 * 1000
#define DSHOT600        600 * 1000
#define DSHOT1200       1200 * 1000

typedef rmt_channel_handle_t DSHOT_Channel_handle_t;
typedef rmt_encoder_handle_t DSHOT_encoder_handle_t;
typedef rmt_sync_manager_handle_t DSHOT_sync_manager_handle_t;

/*
Num     |   Command                                                         |   Remark
0       |   DSHOT_CMD_MOTOR_STOP                                            |   Currently not implemented
1       |   DSHOT_CMD_BEEP1                                                 |   Wait at least length of beep (260ms) before next command
2       |   DSHOT_CMD_BEEP2                                                 |   Wait at least length of beep (260ms) before next command
3       |   DSHOT_CMD_BEEP3                                                 |   Wait at least length of beep (260ms) before next command
4       |   DSHOT_CMD_BEEP4                                                 |   Wait at least length of beep (260ms) before next command
5       |   DSHOT_CMD_BEEP5                                                 |   Wait at least length of beep (260ms) before next command
6       |   DSHOT_CMD_ESC_INFO                                              |   Wait at least 12ms before next command
7       |   DSHOT_CMD_SPIN_DIRECTION_1                                      |   Need 6x
8       |   DSHOT_CMD_SPIN_DIRECTION_2                                      |   Need 6x
9       |   DSHOT_CMD_3D_MODE_OFF                                           |   Need 6x
10      |   DSHOT_CMD_3D_MODE_ON                                            |   Need 6x
11      |   DSHOT_CMD_SETTINGS_REQUEST                                      |   Currently not implemented
12      |   DSHOT_CMD_SAVE_SETTINGS                                         |   Need 6x, wait at least 35ms before next command
13      |   DSHOT_EXTENDED_TELEMETRY_ENABLE                                 |   Need 6x (only on EDT enabled firmware)
14      |   DSHOT_EXTENDED_TELEMETRY_DISABLE                                |   Need 6x (only on EDT enabled firmware)
15      |   -                                                               |   Not yet assigned
16      |   -                                                               |   Not yet assigned
17      |   -                                                               |   Not yet assigned
18      |   -                                                               |   Not yet assigned
19      |   -                                                               |   Not yet assigned
20      |   DSHOT_CMD_SPIN_DIRECTION_NORMAL                                 |   Need 6x
21      |   DSHOT_CMD_SPIN_DIRECTION_REVERSED                               |   Need 6x
22      |   DSHOT_CMD_LED0_ON                                               |   -
23      |   DSHOT_CMD_LED1_ON                                               |   -
24      |   DSHOT_CMD_LED2_ON                                               |   -
25      |   DSHOT_CMD_LED3_ON                                               |   -
26      |   DSHOT_CMD_LED0_OFF                                              |   -
27      |   DSHOT_CMD_LED1_OFF                                              |   -
28      |   DSHOT_CMD_LED2_OFF                                              |   -
29      |   DSHOT_CMD_LED3_OFF                                              |   -
30      |   Audio_Stream mode on/Off                                        |   Currently not implemented
31      |   Silent Mode on/Off                                              |   Currently not implemented
32      |   DSHOT_CMD_SIGNAL_LINE_TELEMETRY_DISABLE                         |   Need 6x. Disables commands 42 to 47
33      |   DSHOT_CMD_SIGNAL_LINE_TELEMETRY_ENABLE                          |   Need 6x. Disables commands 42 to 47
34      |   DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_TELEMETRY                 |   Need 6x. Enables commands 42 to 47 and sends eRPM if normal DShot frame
35      |   DSHOT_CMD_SIGNAL_LINE_CONTINUOUS_ERPM_PERIOD_TELEMETRY          |   Need 6x. Enables commands 42 to 47 and sends eRPM period if normal DShot frame
36      |   -                                                               |   Not yet assigned
37      |   -                                                               |   Not yet assigned
38      |   -                                                               |   Not yet assigned
39      |   -                                                               |   Not yet assigned
40      |   -                                                               |   Not yet assigned
41      |   -                                                               |   Not yet assigned
42      |   DSHOT_CMD_SIGNAL_LINE_TEMPERATURE_TELEMETRY                     |   1°C per LSB
43      |   DSHOT_CMD_SIGNAL_LINE_VOLTAGE_TELEMETRY                         |   10mV per LSB, 40.95V max
44      |   DSHOT_CMD_SIGNAL_LINE_CURRENT_TELEMETRY                         |   100mA per LSB, 409.5A max
45      |   DSHOT_CMD_SIGNAL_LINE_CONSUMPTION_TELEMETRY                     |   10mAh per LSB, 40.95Ah max
46      |   DSHOT_CMD_SIGNAL_LINE_ERPM_TELEMETRY                            |   100erpm per LSB, 409500erpm max
47      |   DSHOT_CMD_SIGNAL_LINE_ERPM_PERIOD_TELEMETRY                     |   16µs per LSB, 65520µs max TBD
*/

/**
 * @brief Throttle representation in DShot protocol
 */
typedef struct {
    uint16_t throttle;  /*!< Throttle value */
    bool telemetry_req; /*!< Telemetry request */
} dshot_esc_throttle_t;

/**
 * @brief Type of Dshot ESC encoder configuration
 */
typedef struct {
    uint32_t resolution;    /*!< Encoder resolution, in Hz */
    uint32_t baud_rate;     /*!< Dshot protocol runs at several different baud rates, e.g. DSHOT300 = 300k baud rate */
    uint32_t post_delay_us; /*!< Delay time after one Dshot frame, in microseconds */
} dshot_esc_encoder_config_t;

/**
 * @brief Create RMT encoder for encoding Dshot ESC frame into RMT symbols
 *
 * @param[in] config Encoder configuration
 * @param[out] ret_encoder Returned encoder handle
 * @return
 *      - ESP_ERR_INVALID_ARG for any invalid arguments
 *      - ESP_ERR_NO_MEM out of memory when creating Dshot ESC encoder
 *      - ESP_OK if creating encoder successfully
 */
esp_err_t rmt_new_dshot_esc_encoder(const dshot_esc_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder);

void create_dshot_channel(int8_t dshot_esc_gpio_num, uint32_t dshot_protocol, DSHOT_Channel_handle_t *esc_channel, DSHOT_encoder_handle_t *dshot_encoder);
void install_dshot_sync_manager(DSHOT_Channel_handle_t tx_channels[], DSHOT_sync_manager_handle_t *synchro);
void enable_dshot_channel(DSHOT_Channel_handle_t esc_channel);
void disable_dshot_channel(DSHOT_Channel_handle_t esc_channel);
void dshot_send(uint16_t esc_throttle, bool telemetry, DSHOT_Channel_handle_t esc_channel, DSHOT_encoder_handle_t dshot_encoder);
void dshot_send_times(uint16_t times, uint16_t esc_throttle, bool telemetry, DSHOT_Channel_handle_t esc_channel, DSHOT_encoder_handle_t dshot_encoder);

#endif
