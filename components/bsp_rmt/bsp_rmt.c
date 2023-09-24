#include <stdio.h>
#include "bsp_rmt.h"

void rmt_init(uint8_t rmt_io,rmt_channel_t rmt_channel)
{
  rmt_config_t rmt_tx_cfg = RMT_DEFAULT_CONFIG_TX(rmt_io,rmt_channel);
  rmt_tx_cfg.clk_div = 2;

  rmt_config(&rmt_tx_cfg);
  rmt_driver_install(rmt_tx_cfg.channel,0,0);
}
