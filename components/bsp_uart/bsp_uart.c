#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "bsp_uart.h"

//Mutex Lock Handle
SemaphoreHandle_t Uart0_Mutex_Lock;
SemaphoreHandle_t Uart1_Mutex_Lock;
SemaphoreHandle_t Uart2_Mutex_Lock;
static const char *TAG = "UART";
TaskHandle_t Uart2_event_Handle;
SemaphoreHandle_t Uart0_data_rec;
SemaphoreHandle_t Uart1_data_rec;
SemaphoreHandle_t Uart2_data_rec;


void uart_init(uart_num_t uart_num, uint32_t _baud_rate, int8_t tx_pin, int8_t rx_pin,
               uint16_t rx_buffer_size, uint16_t tx_buffer_size, uint16_t queue_size, QueueHandle_t *queue_handle)
{
  switch (uart_num)
  {
  case uart_0:
    Uart0_Mutex_Lock = xSemaphoreCreateMutex();
    if(Uart0_Mutex_Lock == NULL)
    {
      ESP_LOGE(TAG, "Mutex Lock Create Fail......");
    }
    xSemaphoreTake(Uart0_Mutex_Lock,portMAX_DELAY);     //Lock
    break;
  case uart_1:
    Uart1_Mutex_Lock = xSemaphoreCreateMutex();
    if(Uart1_Mutex_Lock == NULL)
    {
      ESP_LOGE(TAG, "Mutex Lock Create Fail......");
    }
    xSemaphoreTake(Uart1_Mutex_Lock,portMAX_DELAY);     //Lock
    break;
  case uart_2:
    Uart2_Mutex_Lock = xSemaphoreCreateMutex();
    if(Uart2_Mutex_Lock == NULL)
    {
      ESP_LOGE(TAG, "Mutex Lock Create Fail......");
    }
    xSemaphoreTake(Uart2_Mutex_Lock,portMAX_DELAY);     //Lock
    break;
  default:
    break;
  }

  uart_config_t uart_conf = {
    .baud_rate = _baud_rate,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .source_clk = UART_SCLK_DEFAULT,
    //.rx_flow_ctrl_thresh = 
  };

  uart_param_config(uart_num, &uart_conf);
  uart_set_pin(uart_num, tx_pin,rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  uart_driver_install(uart_num, rx_buffer_size, tx_buffer_size, queue_size, queue_handle,0);

  switch (uart_num)
  {
  case uart_0:
    xSemaphoreGive(Uart0_Mutex_Lock);     //UNLock
    break;
  case uart_1:
    xSemaphoreGive(Uart1_Mutex_Lock);
    break;
  case uart_2:
    xSemaphoreGive(Uart2_Mutex_Lock);
    break;
  default:
    break;
  }
}

void uart_send(uart_num_t uart_num, uint8_t *tx_buffer, uint32_t len)
{
  switch (uart_num)
  {
  case uart_0:
    xSemaphoreTake(Uart0_Mutex_Lock,portMAX_DELAY);     //Lock
    break;
  case uart_1:
    xSemaphoreTake(Uart1_Mutex_Lock,portMAX_DELAY);
    break;
  case uart_2:
    xSemaphoreTake(Uart2_Mutex_Lock,portMAX_DELAY);
    break;
  default:
    break;
  }

  uart_write_bytes(uart_num, tx_buffer,len);

  switch (uart_num)
  {
  case uart_0:
    xSemaphoreGive(Uart0_Mutex_Lock);     //UNLock
    break;
  case uart_1:
    xSemaphoreGive(Uart1_Mutex_Lock);
    break;
  case uart_2:
    xSemaphoreGive(Uart2_Mutex_Lock);
    break;
  default:
    break;
  }
}

uint32_t uart_check_rx_fifo(uart_num_t uart_num)
{
  switch (uart_num)
  {
  case uart_0:
    xSemaphoreTake(Uart0_Mutex_Lock,portMAX_DELAY);     //Lock
    break;
  case uart_1:
    xSemaphoreTake(Uart1_Mutex_Lock,portMAX_DELAY);
    break;
  case uart_2:
    xSemaphoreTake(Uart2_Mutex_Lock,portMAX_DELAY);
    break;
  default:
    break;
  }

  size_t p_size[1];
  uart_get_buffered_data_len(uart_num, p_size);
  return *p_size;

  switch (uart_num)
  {
  case uart_0:
    xSemaphoreGive(Uart0_Mutex_Lock);     //UNLock
    break;
  case uart_1:
    xSemaphoreGive(Uart1_Mutex_Lock);
    break;
  case uart_2:
    xSemaphoreGive(Uart2_Mutex_Lock);
    break;
  default:
    break;
  }
}

void uart_read(uart_num_t uart_num, uint8_t *rx_buffer, uint32_t len)
{
  switch (uart_num)
  {
  case uart_0:
    xSemaphoreTake(Uart0_Mutex_Lock,portMAX_DELAY);     //Lock
    break;
  case uart_1:
    xSemaphoreTake(Uart1_Mutex_Lock,portMAX_DELAY);
    break;
  case uart_2:
    xSemaphoreTake(Uart2_Mutex_Lock,portMAX_DELAY);
    break;
  default:
    break;
  }

  uart_read_bytes(uart_num, rx_buffer, len, 0xFFFF);

  switch (uart_num)
  {
  case uart_0:
    xSemaphoreGive(Uart0_Mutex_Lock);     //UNLock
    break;
  case uart_1:
    xSemaphoreGive(Uart1_Mutex_Lock);
    break;
  case uart_2:
    xSemaphoreGive(Uart2_Mutex_Lock);
    break;
  default:
    break;
  }
}

void uart_rx_fifo_clear(uart_num_t uart_num)
{
  switch (uart_num)
  {
  case uart_0:
    xSemaphoreTake(Uart0_Mutex_Lock,portMAX_DELAY);     //Lock
    break;
  case uart_1:
    xSemaphoreTake(Uart1_Mutex_Lock,portMAX_DELAY);
    break;
  case uart_2:
    xSemaphoreTake(Uart2_Mutex_Lock,portMAX_DELAY);
    break;
  default:
    break;
  }

  uart_flush(uart_num);

  switch (uart_num)
  {
  case uart_0:
    xSemaphoreGive(Uart0_Mutex_Lock);     //UNLock
    break;
  case uart_1:
    xSemaphoreGive(Uart1_Mutex_Lock);
    break;
  case uart_2:
    xSemaphoreGive(Uart2_Mutex_Lock);
    break;
  default:
    break;
  }
}

void uart_mask_intr_enable(uart_num_t uart_num, uint32_t mask)
{
  switch (uart_num)
  {
  case uart_0:
    xSemaphoreTake(Uart0_Mutex_Lock,portMAX_DELAY);     //Lock
    break;
  case uart_1:
    xSemaphoreTake(Uart1_Mutex_Lock,portMAX_DELAY);
    break;
  case uart_2:
    xSemaphoreTake(Uart2_Mutex_Lock,portMAX_DELAY);
    break;
  default:
    break;
  }
  
  uart_enable_intr_mask(uart_num, mask);

  switch (uart_num)
  {
  case uart_0:
    xSemaphoreGive(Uart0_Mutex_Lock);     //UNLock
    break;
  case uart_1:
    xSemaphoreGive(Uart1_Mutex_Lock);
    break;
  case uart_2:
    xSemaphoreGive(Uart2_Mutex_Lock);
    break;
  default:
    break;
  }
}

void uart_mask_intr_disable(uart_num_t uart_num, uint32_t mask)
{
  switch (uart_num)
  {
  case uart_0:
    xSemaphoreTake(Uart0_Mutex_Lock,portMAX_DELAY);     //Lock
    break;
  case uart_1:
    xSemaphoreTake(Uart1_Mutex_Lock,portMAX_DELAY);
    break;
  case uart_2:
    xSemaphoreTake(Uart2_Mutex_Lock,portMAX_DELAY);
    break;
  default:
    break;
  }

  uart_disable_intr_mask(uart_num, mask);

  switch (uart_num)
  {
  case uart_0:
    xSemaphoreGive(Uart0_Mutex_Lock);     //UNLock
    break;
  case uart_1:
    xSemaphoreGive(Uart1_Mutex_Lock);
    break;
  case uart_2:
    xSemaphoreGive(Uart2_Mutex_Lock);
    break;
  default:
    break;
  }
}

void uart_intr_cfg(uart_num_t uart_num, uint32_t mask, uint8_t rx_tout_thresh, uint8_t rx_full_thresh, uint8_t tx_empty_thresh)
{
  switch (uart_num)
  {
  case uart_0:
    xSemaphoreTake(Uart0_Mutex_Lock,portMAX_DELAY);     //Lock
    break;
  case uart_1:
    xSemaphoreTake(Uart1_Mutex_Lock,portMAX_DELAY);
    break;
  case uart_2:
    xSemaphoreTake(Uart2_Mutex_Lock,portMAX_DELAY);
    break;
  default:
    break;
  }

  uart_intr_config_t uart_intr_conf = {
    .intr_enable_mask = mask,
    .rx_timeout_thresh = rx_tout_thresh,
    .rxfifo_full_thresh = rx_full_thresh,
    .txfifo_empty_intr_thresh = tx_empty_thresh,
  };
  uart_intr_config(uart_num, &uart_intr_conf);

  switch (uart_num)
  {
  case uart_0:
    xSemaphoreGive(Uart0_Mutex_Lock);     //UNLock
    break;
  case uart_1:
    xSemaphoreGive(Uart1_Mutex_Lock);
    break;
  case uart_2:
    xSemaphoreGive(Uart2_Mutex_Lock);
    break;
  default:
    break;
  }
}

void uart_rx_intr_enable(uart_num_t uart_num)
{
  switch (uart_num)
  {
  case uart_0:
    xSemaphoreTake(Uart0_Mutex_Lock,portMAX_DELAY);     //Lock
    break;
  case uart_1:
    xSemaphoreTake(Uart1_Mutex_Lock,portMAX_DELAY);
    break;
  case uart_2:
    xSemaphoreTake(Uart2_Mutex_Lock,portMAX_DELAY);
    break;
  default:
    break;
  }

  uart_enable_rx_intr(uart_num);

  switch (uart_num)
  {
  case uart_0:
    xSemaphoreGive(Uart0_Mutex_Lock);     //UNLock
    break;
  case uart_1:
    xSemaphoreGive(Uart1_Mutex_Lock);
    break;
  case uart_2:
    xSemaphoreGive(Uart2_Mutex_Lock);
    break;
  default:
    break;
  }
}

void uart_rx_intr_disable(uart_num_t uart_num)
{
  switch (uart_num)
  {
  case uart_0:
    xSemaphoreTake(Uart0_Mutex_Lock,portMAX_DELAY);     //Lock
    break;
  case uart_1:
    xSemaphoreTake(Uart1_Mutex_Lock,portMAX_DELAY);
    break;
  case uart_2:
    xSemaphoreTake(Uart2_Mutex_Lock,portMAX_DELAY);
    break;
  default:
    break;
  }

  uart_disable_rx_intr(uart_num);

  switch (uart_num)
  {
  case uart_0:
    xSemaphoreGive(Uart0_Mutex_Lock);     //UNLock
    break;
  case uart_1:
    xSemaphoreGive(Uart1_Mutex_Lock);
    break;
  case uart_2:
    xSemaphoreGive(Uart2_Mutex_Lock);
    break;
  default:
    break;
  }
}

void uart_intr_event_serve_create(SemaphoreHandle_t* Semaphore, TaskFunction_t pTask, QueueHandle_t *queue, TaskHandle_t *const pHandle)
{
  // Uart2_data_rec = xSemaphoreCreateBinary();
  // xTaskCreate(uart_event_task, "uart event task", 4096, queue, 12, &Uart_event_Handle);
  *Semaphore = xSemaphoreCreateBinary();
  xTaskCreate(pTask, "uart event task", 4096, queue, 12, pHandle);
}
