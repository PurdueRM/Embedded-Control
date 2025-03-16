#include "bsp_uart.h"

#include <stdlib.h>
#include "memory.h"

#define UART_INSTANCE_MAX 3
UART_Instance_t *g_uart_insatnces[UART_INSTANCE_MAX];
uint8_t g_uart_instance_count = 0;

/**
 * @brief Initialize UART service
 * 
 * @param uart_insatce yes this is a typo, should be uart_instance
 * 
 * @note enable uart receive calling HAL_UARTEx_ReceiveToIdle_DMA. This
 * function will enable uart receive with DMA. There are three interrupts
 * that can be enabled: DMA_IT_TC (DMA transfer complete), DMA_IT_HT (DMA
 * Half Complete), UART_IDLE (UART Idle).
 * 
 * DMA_IT_TC is triggered when the DMA transfer is complete.
 * DMA_IT_HT is triggered when half of the buffer is filled.
 * UART_IDLE is triggered when the UART is idle for a period of time, typically
 * 1 byte time.
 * 
 * Here we only care about the UART_IDLE interrupt, and DMA_IT_TC interrupt.
 * Therefore we disable the DMA_IT_HT interrupt by calling __HAL_DMA_DISABLE_IT. 
*/
void UART_Service_Init(UART_Instance_t *uart_insatce)
{
    // enable uart receive
    HAL_UARTEx_ReceiveToIdle_DMA(uart_insatce->uart_handle, uart_insatce->rx_buffer, uart_insatce->rx_buffer_size);
    // disable half transfer interrupt
    __HAL_DMA_DISABLE_IT(uart_insatce->uart_handle->hdmarx, DMA_IT_HT); // disable half transfer interrupt
}

/**
 * @brief Register UART instance
 * 
 * @param huart UART handle
 * @param rx_buffer buffer to store received data
 * @param rx_buffer_size size of the buffer
 * @param callback callback function when UART receive is complete
*/
UART_Instance_t *UART_Register(UART_HandleTypeDef *huart, uint8_t *rx_buffer, uint16_t rx_buffer_size, void (*callback)(UART_Instance_t *uart_instance))
{
    UART_Instance_t *uart_instance = (UART_Instance_t *)malloc(sizeof(UART_Instance_t));
    uart_instance->uart_handle = huart;
    uart_instance->rx_buffer = rx_buffer;
    uart_instance->rx_buffer_size = rx_buffer_size;
    uart_instance->callback = callback;

    // initialize UART service
    UART_Service_Init(uart_instance);
    
    // store the instance, to iterate through all instances when iterrupt is triggered
    g_uart_insatnces[g_uart_instance_count++] = uart_instance;
    return uart_instance;
}

/**
 * @brief Transmit based on send type
 * 
 * @param uart_instance
 * @param tx_buffer
 * @param tx_buffer_size
 * @param send_type
 *        - UART_BLOCKING: halt CPU until transmission is complete
 *        - UART_IT: use interrupt to transmit data
 *        - UART_DMA: use DMA to transmit data
*/
HAL_StatusTypeDef UART_Transmit(UART_Instance_t *uart_instance, uint8_t *tx_buffer, uint16_t tx_buffer_size, uint8_t send_type)
{
    switch (send_type)
    {
    case UART_BLOCKING:
        return HAL_UART_Transmit(uart_instance->uart_handle, tx_buffer, tx_buffer_size, HAL_MAX_DELAY);
        break;
    case UART_IT:
        return HAL_UART_Transmit_IT(uart_instance->uart_handle, tx_buffer, tx_buffer_size);
        break;
    case UART_DMA:
        return HAL_UART_Transmit_DMA(uart_instance->uart_handle, tx_buffer, tx_buffer_size);
        break;
    default:
        return HAL_ERROR;
        break;
    }
}

/**
 * @brief UART receive callback
 * 
 * @param huart UART handle
 * @param Size size of the received data
 * 
 * @note This function is called when the UART receive is complete. It will
 * iterate through all registered UART instances, and call the callback function
 * if the UART handle matches.
*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // iterate through all registered UART instances
    for (int i = 0; i < g_uart_instance_count; i++)
    {
        // if the UART handle matches
        if (g_uart_insatnces[i]->uart_handle == huart)
        {
            // if the callback function is not NULL
            if (g_uart_insatnces[i]->callback != NULL)
            {
                // call the callback function
                g_uart_insatnces[i]->callback(g_uart_insatnces[i]);

                // enable uart receive for next data frame
                HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart_insatnces[i]->rx_buffer, g_uart_insatnces[i]->rx_buffer_size);
                // still disable half transfer interrupt (@ref void UART_Service_Init(void))
                __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
            }
        }
    }
    
}
