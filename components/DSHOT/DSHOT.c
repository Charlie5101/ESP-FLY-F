#include <stdio.h>
#include "esp_check.h"
#include "DSHOT.h"

#define DSHOT_ESC_RESOLUTION_HZ 40000000

static const char *TAG = "DSHOT";

/**
 * @brief Type of Dshot ESC frame
 */
typedef union
{
  struct
  {
    uint16_t crc : 4;       /*!< CRC checksum */
    uint16_t telemetry : 1; /*!< Telemetry request */
    uint16_t throttle : 11; /*!< Throttle value */
  };
  uint16_t val;
} dshot_esc_frame_t;

#ifndef __cplusplus
_Static_assert(sizeof(dshot_esc_frame_t) == 0x02, "Invalid size of dshot_esc_frame_t structure");
#endif

typedef struct
{
  rmt_encoder_t base;
  rmt_encoder_t *bytes_encoder;
  rmt_encoder_t *copy_encoder;
  rmt_symbol_word_t dshot_delay_symbol;
  int state;
} rmt_dshot_esc_encoder_t;

static void make_dshot_frame(dshot_esc_frame_t *frame, uint16_t throttle, bool telemetry)
{
  frame->throttle = throttle;
  frame->telemetry = telemetry;
  uint16_t val = frame->val;
  uint8_t crc = ((val ^ (val >> 4) ^ (val >> 8)) & 0xF0) >> 4;
  ;
  frame->crc = crc;
  val = frame->val;
  // change the endian
  frame->val = ((val & 0xFF) << 8) | ((val & 0xFF00) >> 8);
}

static size_t rmt_encode_dshot_esc(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                                   const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state)
{
  rmt_dshot_esc_encoder_t *dshot_encoder = __containerof(encoder, rmt_dshot_esc_encoder_t, base);
  rmt_encoder_handle_t bytes_encoder = dshot_encoder->bytes_encoder;
  rmt_encoder_handle_t copy_encoder = dshot_encoder->copy_encoder;
  rmt_encode_state_t session_state = RMT_ENCODING_RESET;
  rmt_encode_state_t state = RMT_ENCODING_RESET;
  size_t encoded_symbols = 0;

  // convert user data into dshot frame
  dshot_esc_throttle_t *throttle = (dshot_esc_throttle_t *)primary_data;
  dshot_esc_frame_t frame = {};
  make_dshot_frame(&frame, throttle->throttle, throttle->telemetry_req);

  switch (dshot_encoder->state)
  {
  case 0: // send the dshot frame
    encoded_symbols += bytes_encoder->encode(bytes_encoder, channel, &frame, sizeof(frame), &session_state);
    if (session_state & RMT_ENCODING_COMPLETE)
    {
      dshot_encoder->state = 1; // switch to next state when current encoding session finished
    }
    if (session_state & RMT_ENCODING_MEM_FULL)
    {
      state |= RMT_ENCODING_MEM_FULL;
      goto out; // yield if there's no free space for encoding artifacts
    }
  // fall-through
  case 1:
    encoded_symbols += copy_encoder->encode(copy_encoder, channel, &dshot_encoder->dshot_delay_symbol,
                                            sizeof(rmt_symbol_word_t), &session_state);
    if (session_state & RMT_ENCODING_COMPLETE)
    {
      state |= RMT_ENCODING_COMPLETE;
      dshot_encoder->state = RMT_ENCODING_RESET; // switch to next state when current encoding session finished
    }
    if (session_state & RMT_ENCODING_MEM_FULL)
    {
      state |= RMT_ENCODING_MEM_FULL;
      goto out; // yield if there's no free space for encoding artifacts
    }
  }
out:
  *ret_state = state;
  return encoded_symbols;
}

static esp_err_t rmt_del_dshot_encoder(rmt_encoder_t *encoder)
{
  rmt_dshot_esc_encoder_t *dshot_encoder = __containerof(encoder, rmt_dshot_esc_encoder_t, base);
  rmt_del_encoder(dshot_encoder->bytes_encoder);
  rmt_del_encoder(dshot_encoder->copy_encoder);
  free(dshot_encoder);
  return ESP_OK;
}

static esp_err_t rmt_dshot_encoder_reset(rmt_encoder_t *encoder)
{
  rmt_dshot_esc_encoder_t *dshot_encoder = __containerof(encoder, rmt_dshot_esc_encoder_t, base);
  rmt_encoder_reset(dshot_encoder->bytes_encoder);
  rmt_encoder_reset(dshot_encoder->copy_encoder);
  dshot_encoder->state = RMT_ENCODING_RESET;
  return ESP_OK;
}

esp_err_t rmt_new_dshot_esc_encoder(const dshot_esc_encoder_config_t *config, rmt_encoder_handle_t *ret_encoder)
{
  esp_err_t ret = ESP_OK;
  rmt_dshot_esc_encoder_t *dshot_encoder = NULL;
  ESP_GOTO_ON_FALSE(config && ret_encoder, ESP_ERR_INVALID_ARG, err, TAG, "invalid argument");
  dshot_encoder = rmt_alloc_encoder_mem(sizeof(rmt_dshot_esc_encoder_t));
  ESP_GOTO_ON_FALSE(dshot_encoder, ESP_ERR_NO_MEM, err, TAG, "no mem for musical score encoder");
  dshot_encoder->base.encode = rmt_encode_dshot_esc;
  dshot_encoder->base.del = rmt_del_dshot_encoder;
  dshot_encoder->base.reset = rmt_dshot_encoder_reset;
  uint32_t delay_ticks = config->resolution / 1e6 * config->post_delay_us;
  rmt_symbol_word_t dshot_delay_symbol = {
      .level0 = 0,
      .duration0 = delay_ticks / 2,
      .level1 = 0,
      .duration1 = delay_ticks / 2,
  };
  dshot_encoder->dshot_delay_symbol = dshot_delay_symbol;
  // different dshot protocol have its own timing requirements,
  float period_ticks = (float)config->resolution / config->baud_rate;
  // 1 and 0 is represented by a 74.850% and 37.425% duty cycle respectively
  unsigned int t1h_ticks = (unsigned int)(period_ticks * 0.7485);
  unsigned int t1l_ticks = (unsigned int)(period_ticks - t1h_ticks);
  unsigned int t0h_ticks = (unsigned int)(period_ticks * 0.37425);
  unsigned int t0l_ticks = (unsigned int)(period_ticks - t0h_ticks);
  rmt_bytes_encoder_config_t bytes_encoder_config = {
      .bit0 = {
          .level0 = 1,
          .duration0 = t0h_ticks,
          .level1 = 0,
          .duration1 = t0l_ticks,
      },
      .bit1 = {
          .level0 = 1,
          .duration0 = t1h_ticks,
          .level1 = 0,
          .duration1 = t1l_ticks,
      },
      .flags.msb_first = 1,
  };
  ESP_GOTO_ON_ERROR(rmt_new_bytes_encoder(&bytes_encoder_config, &dshot_encoder->bytes_encoder), err, TAG, "create bytes encoder failed");
  rmt_copy_encoder_config_t copy_encoder_config = {};
  ESP_GOTO_ON_ERROR(rmt_new_copy_encoder(&copy_encoder_config, &dshot_encoder->copy_encoder), err, TAG, "create copy encoder failed");
  *ret_encoder = &dshot_encoder->base;
  return ESP_OK;
err:
  if (dshot_encoder)
  {
    if (dshot_encoder->bytes_encoder)
    {
      rmt_del_encoder(dshot_encoder->bytes_encoder);
    }
    if (dshot_encoder->copy_encoder)
    {
      rmt_del_encoder(dshot_encoder->copy_encoder);
    }
    free(dshot_encoder);
  }
  return ret;
}

void create_dshot_channel(int8_t dshot_esc_gpio_num, uint32_t dshot_protocol, DSHOT_Channel_handle_t *esc_channel, DSHOT_encoder_handle_t *dshot_encoder)
{
  ESP_LOGI(TAG, "Create DSHOT(RMT TX) channel");
  rmt_tx_channel_config_t tx_chan_config = {
      .clk_src = RMT_CLK_SRC_DEFAULT, // select a clock that can provide needed resolution
      .gpio_num = dshot_esc_gpio_num,
      .mem_block_symbols = 48,
      .resolution_hz = DSHOT_ESC_RESOLUTION_HZ,
      .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
  };
  ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, esc_channel));

  ESP_LOGI(TAG, "Install Dshot ESC encoder");
  dshot_esc_encoder_config_t encoder_config = {
      .resolution = DSHOT_ESC_RESOLUTION_HZ,
      .baud_rate = dshot_protocol, // DSHOT100/300/600/1200 protocol
      .post_delay_us = 50,         // extra delay between each frame
  };
  ESP_ERROR_CHECK(rmt_new_dshot_esc_encoder(&encoder_config, dshot_encoder));
}

void install_dshot_sync_manager(DSHOT_Channel_handle_t tx_channels[], DSHOT_sync_manager_handle_t *synchro)
{
  rmt_sync_manager_config_t synchro_config = {
      .tx_channel_array = tx_channels,
      .array_size = sizeof(tx_channels) / sizeof(tx_channels[0]), //un verify
      // .array_size = 4,
  };
  ESP_ERROR_CHECK(rmt_new_sync_manager(&synchro_config, synchro));
}

void enable_dshot_channel(DSHOT_Channel_handle_t esc_channel)
{
  // ESP_LOGI(TAG, "Enable DSHOT(RMT TX) channel");
  ESP_ERROR_CHECK(rmt_enable(esc_channel));
}

void disable_dshot_channel(DSHOT_Channel_handle_t esc_channel)
{
  // ESP_LOGI(TAG, "Disable DSHOT(RMT TX) channel");
  ESP_ERROR_CHECK(rmt_disable(esc_channel));
}

void dshot_send(uint16_t esc_throttle, DSHOT_Channel_handle_t esc_channel, DSHOT_encoder_handle_t dshot_encoder)
{
  //refresh
  rmt_disable(esc_channel);
  rmt_enable(esc_channel);
  rmt_transmit_config_t tx_config = {
      .loop_count = -1, // infinite loop
  };
  dshot_esc_throttle_t throttle = {
      .throttle = esc_throttle,
      .telemetry_req = false,
  };
  ESP_ERROR_CHECK(rmt_transmit(esc_channel, dshot_encoder, &throttle, sizeof(throttle), &tx_config));
}
