#include "bsp_uart.h"

#include <stdlib.h>
#include "memory.h"
#include "portmacro.h"

#define UART_INSTANCE_MAX 3
UART_Instance_t *g_uart_instances[UART_INSTANCE_MAX];
uint8_t g_uart_instance_count = 0;


/** 
 * @brief Find registered UART instance
 * 
 * @param huart UART handle
*/
static UART_Instance_t* get_uart_instance(UART_HandleTypeDef *huart) {
    for (int i = 0; i < g_uart_instance_count; i++) {
        if (g_uart_instances[i]->uart_handle == huart) {
            return g_uart_instances[i];
        }
    }
    return NULL;
}

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

    // FreeRTOS tx semaphore
    uart_instance->tx_complete_sem = xSemaphoreCreateBinary();
    
    // store the instance, to iterate through all instances when iterrupt is triggered
    g_uart_instances[g_uart_instance_count++] = uart_instance;
    return uart_instance;
}

/**
 * @brief Enables sending over UART
 *
 * @param uart_instance UART instance
 * @param tx_buffer Pointer to array of data to send
 * @param tx_buffer_size Length of tx buffer
 * @param timeout Longest time allowed for task to be blocked. Make sure it is longer than transmit time.
 *
 * @note The timeout parameter is there so if the transmit fails, the
 * sending task is not blocked forever. The formula for how long the timeout
 * should be is (Number of Bytes * 10 * 1000) / Baud Rate + a safety margin of ~5 ms
 * to account for contex
*/
HAL_StatusTypeDef UART_Transmit(UART_Instance_t *uart_instance, uint8_t *tx_buffer, uint16_t tx_buffer_size, TickType_t timeout)
{
    // switch (send_type)
    // {
    // case UART_BLOCKING:
    //     return HAL_UART_Transmit(uart_instance->uart_handle, tx_buffer, tx_buffer_size, HAL_MAX_DELAY);
    //     break;
    // case UART_IT:
    //     return HAL_UART_Transmit_IT(uart_instance->uart_handle, tx_buffer, tx_buffer_size);
    //     break;
    // case UART_DMA:
    //     return HAL_UART_Transmit_DMA(uart_instance->uart_handle, tx_buffer, tx_buffer_size);
    //     break;
    // default:
    //     return HAL_ERROR;
    //     break;
    // }
    if (uart_instance == NULL || tx_buffer == NULL || tx_buffer_size <= 0) {
        return HAL_ERROR;
    }

    // Clear any old semaphore state just in case
    xSemaphoreTake(uart_instance->tx_complete_sem, 0);

    // Tell HAL to start the DMA
    if (HAL_UART_Transmit_DMA(uart_instance->uart_handle, tx_buffer, tx_buffer_size) != HAL_OK) {
        return HAL_ERROR;
    }

    // TickType_t timeout = 6;
    
    // Put calling task to sleep until tx callback wakes it up or timeout is reached
    if (xSemaphoreTake(uart_instance->tx_complete_sem, timeout) != pdTRUE) {
        // Condition only true if hardware timed out or froze
        HAL_UART_AbortTransmit(uart_instance->uart_handle); 
        return HAL_ERROR; 
    }

    return HAL_OK;
}

/**
 * @brief UART transmit callback
 *
 * @param huart UART handle
 *
 * @note This callback is called when a transmit finished,
 * then unblocks the task which called UART_Send.
*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    // Find UART instance
    UART_Instance_t *instance = get_uart_instance(huart);
    if (instance == NULL) {
        return;
    }

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Wake up the sending task
    xSemaphoreGiveFromISR(instance->tx_complete_sem, &xHigherPriorityTaskWoken);
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
        if (g_uart_instances[i]->uart_handle == huart)
        {
            // if the callback function is not NULL
            if (g_uart_instances[i]->callback != NULL)
            {
                // call the callback function
                g_uart_instances[i]->callback(g_uart_instances[i]);

                // enable uart receive for next data frame
                HAL_UARTEx_ReceiveToIdle_DMA(huart, g_uart_instances[i]->rx_buffer, g_uart_instances[i]->rx_buffer_size);
                // still disable half transfer interrupt (@ref void UART_Service_Init(void))
                __HAL_DMA_DISABLE_IT(huart3.hdmarx, DMA_IT_HT);
            }
        }
    }
    
}
