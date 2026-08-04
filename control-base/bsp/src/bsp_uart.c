#include "bsp_uart.h"

#include <stdlib.h>
#include "memory.h"

#include "FreeRTOS.h"
#include "stream_buffer.h"

#define UART_INSTANCE_MAX 4
UART_Instance_t *g_uart_insatnces[UART_INSTANCE_MAX];
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
 * @brief Register UART instance
 * 
 * @param huart UART handle
 * @param rx_buffer buffer to store received data
 * @param rx_buffer_size size of the buffer
 * @param callback callback function when UART receive is complete
*/
UART_Instance_t *UART_Register(UART_HandleTypedef *huart, uint8_t *rx_buffer, uint16_T rx_buffer_size, size_t trigger_level) {
    if (g_uart_instance_count >= UART_INSTANCE_MAX) { // Overflow protection
        return NULL; 
    }
    // Uses thread safe FreeRTOS malloc
    UART_Instance_t *uart_instance = (UART_Instance_t *) pvPortMalloc(sizeof(UART_Instance_t));
    if (uart_instance == NULL) { 
        return NULL; 
    }
    
    uart_instance->uart_handle = huart;
    uart_instance->rx_buffer = rx_buffer;
    uart_instance->rx_buffer_size = rx_buffer_size;
    uart_instance->read_ptr = 0;
    uart_instance->error_count = 0;

    // Create stream buffer 2x the size of rx buffer for safety margin (can handle two full messages)
    uart_instance->stream_buffer = xStreamBufferCreate(rx_buffer_size * 2, trigger_level);

    if (uart_instance->stream_buffer == NULL) {
        vPortFree(uart_instance); // Clean up if RTOS heap is full
        return NULL;
    }

    uart_instance->is_initialized = 1;

    // Store the instance, to iterate through all instances when iterrupt is triggered
    g_uart_instances[g_uart_instance_count++] = uart_instance;

    // starts DMA
    HAL_UARTEx_ReceiveToIdle_DMA(huart, rx_buffer, rx_buffer_size);

    return uart_instance;
}

/**
 * @brief UART receive callback
 * 
 * @param huart UART handle
 * @param Size size of the received data
 * 
 * @note This function is called after three different  interrupts:
 * - DMA_IT_HT (Half Transfer Interrupt) --> DMA fills buffer to 50%
 * - DMA_IT_TC (Transfer Complete Interrupt) --> DMA fully fills buffer
 * - DMA_IT_IDLE (Idle Interrupt) --> Line is idle (high) for one frame (size of DMA buffer)
 * We enable HT so if we have a continuous stream of bytes (IDLE is never called)
 * then we have time to send data to the stream before it is overwritten by the cyclical DMA.
*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // Find UART instance
    UART_Instance_t *instance = get_uart_instance(huart);
    if (instance == NULL || !instance->is_initialized) {
        return;
    }

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint16_t write_ptr = Size;
    uint16_t len = 0;

    // No new data
    if (write_ptr == instance->read_ptr) {
        return;
    }

    // Calculate how much new data since the last read
    if (write_ptr > instance->read_ptr) { // Write pointer ahead of read pointer
        len = write_ptr - instance->read_ptr;

        // Send data to FreeRTOS stream buffer
        xStreamBufferSendFromISR(instance->stream_buffer, 
                                 &instance->rx_buffer[instance->read_ptr], 
                                 len, 
                                 &xHigherPriorityTaskWoken);
    }
    else { // Wrapped around case (transfer complete interrupt)
        len = instance->rx_buffer_size - instance->read_ptr;

        // Read from the end of rx buffer to stream buffer
        xStreamBufferSendFromISR(instance->stream_buffer, 
                                 &instance->rx_buffer[instance->read_ptr], 
                                 len, 
                                 &xHigherPriorityTaskWoken);

        // Read from the beginning of rx buffer to write ptr
        if (write_ptr > 0) {
            xStreamBufferSendFromISR(instance->stream_buffer, 
                                     &instance->rx_buffer[0], 
                                     write_ptr, 
                                     &xHigherPriorityTaskWoken);
        }
    }

    // Update pointer
    instance->read_ptr = write_ptr;
    if (instance->read_ptr == instance->rx_buffer_size) {
        instance->read_ptr = 0;
    }

    // Force a context switch if a higher priority task was unblocked
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    // Find instance 
    UART_Instance_t *instance = get_uart_instance(huart);
    if (instance == NULL || !instance->is_initialized) {
        return;
    }
    
    // Error code handling
    uint32_t err_mask = huart->ErrorCode;
    instance->errors->last_error_code = err_mask;    
    instance->errors->total_errors++;

    if (err_mask & HAL_UART_ERROR_PE) {
        instance->err_parity++;
    }
    if (err_mask & HAL_UART_ERROR_ORE) {
        instance->err_overrun++;
    }
    if (err_mask & HAL_UART_ERROR_FE) {
        instance->err_framing++;
    }
    if (err_mask & HAL_UART_ERROR_NE) {
        instance->err_noise++;
    }
    if (err_mask & HAL_UART_ERROR_DMA) {
        instance->err_dma++;
    }

    // Aborts current reception + clears hardware error flags and resets UART and DMA
    HAL_UART_AbortReceive(huart);

    // Resets tracking pointer to beginning to not read garbage info
    instance->read_ptr = 0;

    // Restart DMA
    HAL_UARTEx_ReceiveToIdle_DMA(huart, instance->rx_buffer, instance->rx_buffer_size);

}
