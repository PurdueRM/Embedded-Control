#ifndef BSP_UART_H
#define BSP_UART_H

#include "usart.h"

#define UART_BLOCKING (0b00000000)
#define UART_IT (0b00000001)
#define UART_DMA (0b00000010)

typedef struct _UART_Instance
{
    UART_HandleTypeDef *uart_handle;
    uint8_t *rx_buffer;
    uint16_t rx_buffer_size;
    void (*callback)(struct _UART_Instance *uart_instance);
} UART_Instance_t;

void UART_Service_Init(UART_Instance_t *uart_insatce);
UART_Instance_t *UART_Register(UART_HandleTypeDef *huart, uint8_t *rx_buffer, uint16_t rx_buffer_size, void (*callback)(UART_Instance_t *uart_instance)); 
HAL_StatusTypeDef UART_Transmit(UART_Instance_t *uart_instance, uint8_t *tx_buffer, uint16_t tx_buffer_size, uint8_t send_type);
#endif // BSP_UART_H