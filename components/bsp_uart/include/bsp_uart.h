#include "driver/uart.h"

void uart_init(uint8_t uart_num,uint32_t _baud_rate,uint8_t tx_pin,uint8_t rx_pin,
               uint16_t rx_buffer_size,uint16_t tx_buffer_size,uint16_t queue_size,QueueHandle_t *queue_handle);
void uart_send(uint8_t uart_num,uint8_t *tx_buffer,uint32_t len);
uint32_t uart_check_rx_fifo(uint8_t uart_num);
void uart_read(uint8_t uart_num,uint8_t *rx_buffer,uint32_t len);
void uart_rx_fifo_clear(uint8_t uart_num);
