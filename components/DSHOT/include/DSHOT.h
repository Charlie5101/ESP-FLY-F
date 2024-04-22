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
void install_dshot_sync_manager(DSHOT_Channel_handle_t *tx_channels, DSHOT_sync_manager_handle_t *synchro);
void enable_dshot_channel(DSHOT_Channel_handle_t esc_channel);
void disable_dshot_channel(DSHOT_Channel_handle_t esc_channel);
void dshot_send(uint16_t esc_throttle,DSHOT_Channel_handle_t esc_channel,DSHOT_encoder_handle_t dshot_encoder);

#endif
