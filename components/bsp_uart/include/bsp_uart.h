#ifndef BSP_UART__
#define BSP_UART__

#include "driver/uart.h"

#define UART1_TX_PIN 17
#define UART1_RX_PIN 18
#define UART2_TX_PIN 19
#define UART2_RX_PIN 20

#define UART_RD_BUFF_SIZE   512
#define EX_UART_NUM   uart_2

/*
• UART_AT_CMD_CHAR_DET_INT：当接收器检测到 AT_CMD 字符时触发此中断。
• UART_RS485_CLASH_INT：在 RS485 模式下检测到发送器和接收器之间的冲突时触发此中断。
• UART_RS485_FRM_ERR_INT：在 RS485 模式下检测到发送块发送的数据帧错误时触发此中断。
• UART_RS485_PARITY_ERR_INT：在 RS485 模式下检测到发送块发送的数据校验位错误时触发此中断。
• UART_TX_DONE_INT：当发送器发送完 FIFO 中的所有数据时触发此中断。
• UART_TX_BRK_IDLE_DONE_INT：发送器发送完最后一个数据后保持空闲状态时触发此中断。标记为空闲状态的最短时间由阈值决定（可配置）。
• UART_TX_BRK_DONE_INT：当发送 FIFO 中的数据发送完之后发送器完成了发送 NULL 则触发此中断。
• UART_GLITCH_DET_INT：当接收器在起始位的中点处检测到毛刺时触发此中断。
• UART_SW_XOFF_INT：UART_SW_FLOW_CON_EN 置位时，当接收器接收到 XOFF 字符时触发此中断。
• UART_SW_XON_INT：UART_SW_FLOW_CON_EN 置位时，当接收器接收到 XON 字符时触发此中断。
• UART_RXFIFO_TOUT_INT：当接收器接收一个字节的时间大于 UART_RX_TOUT_THRHD 时触发此中断。
• UART_BRK_DET_INT：当接收器在停止位之后检测到一个 NULL （即传输一个 NULL 的时间内保持逻辑低电平）时触发此中断。
• UART_CTS_CHG_INT：当接收器检测到 CTSn 信号的沿变化时触发此中断。
• UART_DSR_CHG_INT：当接收器检测到 DSRn 信号的沿变化时触发此中断。
• UART_RXFIFO_OVF_INT：当接收器接收到的数据量多于 FIFO 的存储量时触发此中断。
• UART_FRM_ERR_INT：当接收器检测到数据帧错误时触发此中断。
• UART_PARITY_ERR_INT：当接收器检测到校验位错误时触发此中断。
• UART_TXFIFO_EMPTY_INT：当发送 FIFO 中的数据量少于 UART_TXFIFO_EMPTY_THRHD 所指定的值时触发此中断。
• UART_RXFIFO_FULL_INT：当接收器接收到的数据多于 UART_RXFIFO_FULL_THRHD 所指定的值时触发此中断。
• UART_WAKEUP_INT：UART 被唤醒时产生此中断。

bit Reset
00  0     UART_RXFIFO_FULL_INT_ENA UART_RXFIFO_FULL_INT 的使能位。(R/W)
01  0     UART_TXFIFO_EMPTY_INT_ENA UART_TXFIFO_EMPTY_INT 的使能位。(R/W)
02  0     UART_PARITY_ERR_INT_ENA UART_PARITY_ERR_INT 的使能位。(R/W)
03  0     UART_FRM_ERR_INT_ENA UART_FRM_ERR_INT 的使能位。(R/W)
04  0     UART_RXFIFO_OVF_INT_ENA UART_RXFIFO_OVF_INT 的使能位。(R/W)
05  0     UART_DSR_CHG_INT_ENA UART_DSR_CHG_INT 的使能位。(R/W)
06  0     UART_CTS_CHG_INT_ENA UART_CTS_CHG_INT 的使能位。(R/W)
07  0     UART_BRK_DET_INT_ENA UART_BRK_DET_INT 的使能位。(R/W)
08  0     UART_RXFIFO_TOUT_INT_ENA UART_RXFIFO_TOUT_INT 的使能位。(R/W)
09  0     UART_SW_XON_INT_ENA UART_SW_XON_INT 的使能位。(R/W)
10  0     UART_SW_XOFF_INT_ENA UART_SW_XOFF_INT 的使能位。(R/W)
11  0     UART_GLITCH_DET_INT_ENA UART_GLITCH_DET_INT 的使能位。(R/W)
12  0     UART_TX_BRK_DONE_INT_ENA UART_TX_BRK_DONE_INT 的使能位。(R/W)
13  0     UART_TX_BRK_IDLE_DONE_INT_ENA UART_TX_BRK_IDLE_DONE_INT 的使能位。(R/W)
14  0     UART_TX_DONE_INT_ENA UART_TX_DONE_INT 的使能位。(R/W)
15  0     UART_RS485_PARITY_ERR_INT_ENA UART_RS485_PARITY_ERR_INT 的使能位。(R/W)
16  0     UART_RS485_FRM_ERR_INT_ENA UART_RS485_PARITY_ERR_INT 的使能位。(R/W)
17  0     UART_RS485_CLASH_INT_ENA UART_RS485_CLASH_INT 的使能位。(R/W)
18  0     UART_AT_CMD_CHAR_DET_INT_ENA UART_AT_CMD_CHAR_DET_INT 的使能位。(R/W)
19  0     UART_WAKEUP_INT_ENA UART_WAKEUP_INT 的使能位。(R/W)
*/

extern TaskHandle_t Uart2_event_Handle;
extern SemaphoreHandle_t Uart0_data_rec;
extern SemaphoreHandle_t Uart1_data_rec;
extern SemaphoreHandle_t Uart2_data_rec;

typedef enum
{
  uart_0,
  uart_1,
  uart_2
}uart_num_t;

void uart_init(uart_num_t uart_num, uint32_t _baud_rate, int8_t tx_pin, int8_t rx_pin,
               uint16_t rx_buffer_size, uint16_t tx_buffer_size, uint16_t queue_size, QueueHandle_t *queue_handle);
void uart_send(uart_num_t uart_num, uint8_t *tx_buffer, uint32_t len);
uint32_t uart_check_rx_fifo(uart_num_t uart_num);
void uart_read(uart_num_t uart_num, uint8_t *rx_buffer, uint32_t len);
void uart_rx_fifo_clear(uart_num_t uart_num);
void uart_mask_intr_enable(uart_num_t uart_num, uint32_t mask);
void uart_mask_intr_disable(uart_num_t uart_num, uint32_t mask);
void uart_intr_cfg(uart_num_t uart_num, uint32_t mask, uint8_t rx_tout_thresh, uint8_t rx_full_thresh, uint8_t tx_empty_thresh);
void uart_rx_intr_enable(uart_num_t uart_num);
void uart_rx_intr_disable(uart_num_t uart_num);
void uart_event_task(QueueHandle_t *queue);
// void uart_intr_event_serve_create(QueueHandle_t *queue);
void uart_intr_event_serve_create(SemaphoreHandle_t* Semaphore, TaskFunction_t pTask, QueueHandle_t *queue, TaskHandle_t *const pHandle);


#endif
