#include "bsp_uart.h"

#include <cstdint>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "memory.h"

#include "FreeRTOS.h"
#include "portable.h"
#include "stream_buffer.h"

#define UART_INSTANCE_MAX 4

UART_Instance_t *g_uart_instances[UART_INSTANCE_MAX];
uint8_t g_uart_instance_count = 0;

// Statically allocated buffers

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
 * @param uart_instance Pointer to a statically allocated UART instance
 * @param huart UART handle
 * @param rx_buffer_block Statically allocated block of memory in non-cacheable memory
 * @param rx_buffer_size size of the buffer in bytes
 * @retval true on successful register, false on fail
 * @note This UART implementation only handles fixed-length packets 
*/
uint8_t *UART_Register(UART_Instance_t *uart_instance, UART_HandleTypedef *huart, uint8_t *rx_buffer_block, uint16_t rx_buffer_size) {
    if (g_uart_instance_count >= UART_INSTANCE_MAX || uart_instance == NULL || rx_buffer_block == NULL) { // Overflow protection
        return false; 
    }

    // Initialize state
    memset(uart_instance, 0, sizeof(UART_Instance_t));
    uart_instance->uart_handle = huart;
    uart_instance->rx_buffer_size = rx_buffer_size;
    uart_instance->active_buffer_index = 0;
    uart_instance->read_ptr = 0;

    // Split up buffer block into pointer array
    for (int i = 0; i < UART_MSG_QUEUE_SIZE; i++) {
        uart_instance->rx_buffers[i] = rx_buffer_block + (i * rx_buffer_size);
    }

    // Create FreeRTOS rx message queue
    uart_instance->rx_msg_queue = xQueueCreate(UART_MSG_QUEUE_SIZE, sizeof(UART_Message_t));

    // Create FreeRTOS tx semaphore
    uart_instance->tx_complete_sem = xSemaphoreCreateBinary();

    uart_instance->is_initialized = 1;

    if (uart_instance->msg_queue == NULL || uart_instance->tx_complete_sem == NULL) {
        return false;
    }

    // Store the instance in order to iterate through all instances when rx iterrupt is triggered
    g_uart_instances[g_uart_instance_count++] = uart_instance;

    // starts DMA on buffer 0
    HAL_UARTEx_ReceiveToIdle_DMA(
        uart_instance->uart_handle, 
        uart_instance->rx_buffers[uart_instance->active_buffer_index], 
        uart_instance->rx_buffer_size
    );

    // Disable HT interrupt
    __HAL_DMA_DISABLE_IT(uart_instance->uart_handle->hdmarx, DMA_IT_HT);

    return true;
}

/**
 * @brief UART receive callback
 * 
 * @param huart UART handle
 * @param Size size of the received data
 * 
 * @note This function is called after two different interrupts:
 * - DMA_IT_TC (Transfer Complete Interrupt) --> DMA fully fills buffer
 * - DMA_IT_IDLE (Idle Interrupt) --> Line is idle (high) for one UART frame
 * It creates a message from the rx buffer that was just filled, then sends it
 * into the FreeRTOS queue associated with the uart instance.
*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
    // Find UART instance
    UART_Instance_t *instance = get_uart_instance(huart);
    if (instance == NULL || !instance->is_initialized) {
        return;
    }

    // Figure out what buffer just finished receiving data
    uint8_t finished_buffer_index = instance->active_buffer_index;

    // Flip index
    instance->active_buffer_index = (instance->active_buffer_index + 1) % UART_MSG_QUEUE_SIZE;

    // Restart DMA into new buffer, keeps reading data while ISR is running
    HAL_UARTEx_ReceiveToIdle_DMA(
        huart, 
        instance->rx_buffers[instance->active_buffer_index], 
        instance->rx_buffer_size
    );
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);

    // Create message
    UART_Message_t msg;
    msg.payload = instance->rx_buffers[finished_buffer_index];
    msg.length = Size;
    
    // Send message
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(instance->msg_queue, &msg, &xHigherPriorityTaskWoken);

    // Force a context switch if a higher priority task was unblocked
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
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
    if (instance == NULL || !instance->is_initialized) {
        return;
    }

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    // Wake up the sending task
    xSemaphoreGiveFromISR(instance->tx_complete_sem, &xHigherPriorityTaskWoken);
    
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief Enables sending over UART
 *
 * @param instance UART instance
 * @param tx_buffer Pointer to array of data to send
 * @param tx_buffer_len Length of tx buffer
 * @param timeout Longest time allowed for task to be blocked. Make sure it is longer than transmit time.
 *
 * @note The timeout parameter is there so if the transmit fails, the
 * sending task is not blocked forever. The formula for how long the timeout
 * should be is (Number of Bytes * 10 * 1000) / Baud Rate + a safety margin of ~5 ms
 * to account for context switching and any minor jitter.
*/
uint8_t UART_Transmit(UART_Instance_t *instance, uint8_t *tx_buffer, uint16_t tx_buffer_len, TickType_t timeout) {
    if (instance == NULL || !instance->is_initialized || tx_buffer == NULL || tx_buffer_len <= 0) {
        return false;
    }

    // Clear any old semaphore state just in case
    xSemaphoreTake(instance->tx_complete_sem, 0);

    // Tell HAL to start the DMA
    if (HAL_UART_Transmit_DMA(instance->uart_handle, tx_buffer, tx_buffer_len) != HAL_OK) {
        return false;
    }

    // Put calling task to sleep until tx callback wakes it up or timeout is reached
    if (xSemaphoreTake(instance->tx_complete_sem, timeout) != pdTRUE) {
        // Condition only true if hardware timed out or froze
        HAL_UART_AbortTransmit(instance->uart_handle); 
        return false; 
    }

    return true;
}

/**
 * @brief UART error callback
 *
 * @param huart UART handle
 *
 * @note Since this function aborts the recieve, 
*/
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    // Find instance 
    UART_Instance_t *instance = get_uart_instance(huart);
    if (instance == NULL || !instance->is_initialized) {
        return;
    }
    
    // Error code handling
    uint32_t err_mask = huart->ErrorCode;
    instance->errors.last_error_code = err_mask;    
    instance->errors.total_errors++;

    if (err_mask & HAL_UART_ERROR_PE) {
        instance->errors.err_parity++;
    }
    if (err_mask & HAL_UART_ERROR_ORE) {
        instance->errors.err_overrun++;
    }
    if (err_mask & HAL_UART_ERROR_FE) {
        instance->errors.err_framing++;
    }
    if (err_mask & HAL_UART_ERROR_NE) {
        instance->errors.err_noise++;
    }
    if (err_mask & HAL_UART_ERROR_DMA) {
        instance->errors.err_dma++;
    }

    // Aborts current reception + clears hardware error flags and resets UART and DMA
    HAL_UART_AbortReceive(huart);

    // Resets tracking pointer to beginning to not read garbage info
    instance->read_ptr = 0;

    // Restart DMA
    HAL_UARTEx_ReceiveToIdle_DMA(
        huart, 
        instance->rx_buffers[instance->active_buffer_index], 
        instance->rx_buffer_size
    );
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);

}
