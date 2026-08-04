#include "bsp_uart.h"

#include <stdlib.h>
#include "memory.h"

#include "FreeRTOS.h"
#include "stream_buffer.h"

#define UART_INSTANCE_MAX 4
UART_Instance_t *g_uart_insatnces[UART_INSTANCE_MAX];
uint8_t g_uart_instance_count = 0;

/**
 * @brief Initialize UART service
 * 
 * @param uart_instance
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
 * We use a circular DMA buffer, switching between the first half and second
 * half of the buffer. Thus, the buffer is twice the length of the message
 * being sent.
*/
// void UART_Service_Init(UART_Instance_t *uart_instance)
// {
//     // enable uart receive
//     HAL_UARTEx_ReceiveToIdle_DMA(uart_instance->uart_handle, uart_instance->rx_buffer, uart_instance->rx_buffer_size);
    
// }

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
 * @note This function is called when the UART receive is complete. It will
 * call the callback function if the UART handle has a match. Safely handles HT, 
 * TC, and IDLE interrupts and implements a circular buffer.
*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // Find UART instance
    UART_Instance_t *instance = get_uart_instance(huart);
    if (instance == NULL || instance->is_initialized == 0) {
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