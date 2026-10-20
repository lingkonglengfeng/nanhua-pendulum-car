#ifndef APP_TELEMETRY_H
#define APP_TELEMETRY_H

#include <stdint.h>

/* F103 USART3 TX=PB10, RX=PB11, 115200 8N1.
 * TX DMA1 channel 2; circular RX DMA1 channel 3; ASCII commands. */
void AppTelemetry_Init(void);
void AppTelemetry_Capture(void);
void AppTelemetry_Task(void *argument);
void DMA1_Channel2_IRQHandler(void);
void DMA1_Channel3_IRQHandler(void);
void USART3_IRQHandler(void);

/* Master permission: 1 logs only during BAL_ACTIVE; 0 suppresses periodic logs.
 * GET also returns parameters only during active balance. RX/ACK remain alive. */
extern volatile uint8_t g_uart_log_enable;
/* Default 50Hz compact balance trace; still requires BAL_ACTIVE and LOG=1. */
extern volatile uint8_t g_uart_bal_trace;
extern volatile uint8_t g_uart_log_ready;
extern volatile uint8_t g_uart_log_task_ok;
extern volatile uint8_t g_uart_log_busy;
extern volatile uint32_t g_uart_log_period_ms;
extern volatile uint32_t g_uart_log_packets;
extern volatile uint32_t g_uart_log_dropped;
extern volatile uint32_t g_uart_log_dma_errors;
extern volatile uint32_t g_uart_log_dma_timeouts;
extern volatile uint32_t g_uart_log_stack_free_bytes;
extern volatile uint32_t g_uart_rx_bytes;
extern volatile uint32_t g_uart_rx_overruns;
extern volatile uint32_t g_uart_rx_dma_errors;
extern volatile uint32_t g_uart_rx_uart_errors;

#endif
