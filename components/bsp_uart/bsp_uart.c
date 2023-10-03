#include <stdio.h>
#include "bsp_uart.h"

void uart_init(uint8_t uart_num,uint32_t _baud_rate,uint8_t tx_pin,uint8_t rx_pin,
               uint16_t rx_buffer_size,uint16_t tx_buffer_size,uint16_t queue_size,QueueHandle_t *queue_handle)
{
  uart_config_t uart_conf = {
    .baud_rate = _baud_rate,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    //.rx_flow_ctrl_thresh = 
  };

  uart_param_config(uart_num,&uart_conf);
  uart_set_pin(uart_num,tx_pin,rx_pin,UART_PIN_NO_CHANGE,UART_PIN_NO_CHANGE);
  uart_driver_install(uart_num,rx_buffer_size,tx_buffer_size,queue_size,queue_handle,0);
}

void uart_send(uint8_t uart_num,uint8_t *tx_buffer,uint32_t len)
{
  uart_write_bytes(uart_num,tx_buffer,len);
}

uint32_t uart_check_rx_fifo(uint8_t uart_num)
{
  size_t p_size[1];
  uart_get_buffered_data_len(uart_num,p_size);
  return *p_size;
}

void uart_read(uint8_t uart_num,uint8_t *rx_buffer,uint32_t len)
{
  uart_read_bytes(uart_num,rx_buffer,len,0xFFFF);
}

void uart_rx_fifo_clear(uint8_t uart_num)
{
  uart_flush(uart_num);
}

