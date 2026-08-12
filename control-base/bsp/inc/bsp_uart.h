#ifndef BSP_UART_H
#define BSP_UART_H

#include "usart.h"
// #include "stm32h7xx_hal_usart.h"
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"

#define UART_BLOCKING (0b00000000)
#define UART_IT (0b00000001)
#define UART_DMA (0b00000010)

#define UART_MSG_QUEUE_SIZE 3

typedef struct {
    volatile uint32_t last_error_code;   // Stores the raw HAL error bitmask
    volatile uint32_t total_errors;      // Total number of times the callback fired
    volatile uint32_t err_parity;        // PE: Hardware parity mismatch
    volatile uint32_t err_overrun;       // ORE: CPU/DMA was too slow, bytes were dropped
    volatile uint32_t err_framing;       // FE: Noise or baud rate mismatch broke the stop bit
    volatile uint32_t err_noise;         // NE: Hardware detected noise on the RX line
    volatile uint32_t err_dma;           // DMA-specific transfer errors
    volatile uint32_t err_misaligned;    // Detected fragmented data
} UART_Error_t;

// Message to be sent through FreeRTOS Queue
typedef struct {
    uint8_t *payload;
    uint16_t length;
} UART_Message_t;

typedef struct _UART_Instance
{
    UART_HandleTypeDef *uart_handle;

    // Error handling
    UART_Error_t errors;

    // FreeRTOS
    QueueHandle_t msg_queue;
    StaticQueue_t queue_cb; // Holds queue state

    // TX synchronization
    SemaphoreHandle_t tx_complete_sem;

    // RX buffer tracking
    uint8_t *rx_buffers[UART_MSG_QUEUE_SIZE];
    uint16_t rx_buffer_size;
    uint8_t active_buffer_index;
    uint16_t read_ptr;

    uint8_t is_initialized;

} UART_Instance_t;

// void UART_Service_Init(UART_Instance_t *uart_insatce); -- was used for restarting UART, not needed anymore

uint8_t UART_Register(UART_Instance_t *uart_instance, UART_HandleTypeDef *huart, uint8_t *rx_buffer_block, uint16_t rx_buffer_size);

uint8_t UART_Transmit(UART_Instance_t *instance, uint8_t *tx_buffer, uint16_t tx_buffer_len, TickType_t timeout);

#endif // BSP_UART_H