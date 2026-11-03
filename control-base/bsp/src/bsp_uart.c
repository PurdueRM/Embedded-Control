#include "bsp_uart.h"

#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include "stm32h7xx_hal_uart.h"

#define UART_INSTANCE_MAX 4

UART_Instance_t *g_uart_instances[UART_INSTANCE_MAX];
uint8_t g_uart_instance_count = 0;

// Statically allocated buffers
UART_Instance_t uart1_instance;
__attribute__((section(".non_cacheable"))) UART_Packet_t uart1_rx_buffers[UART_RX_BUF_COUNT];

UART_Instance_t uart2_instance; // RS458
__attribute__((section(".non_cacheable"))) UART_Packet_t uart2_rx_buffers[UART_RX_BUF_COUNT];

UART_Instance_t uart3_instance; // RS458
__attribute__((section(".non_cacheable"))) UART_Packet_t uart3_rx_buffers[UART_RX_BUF_COUNT];

UART_Instance_t uart5_instance;
__attribute__((section(".non_cacheable"))) UART_Packet_t uart5_rx_buffers[UART_RX_BUF_COUNT];

UART_Instance_t uart7_instance;
__attribute__((section(".non_cacheable"))) UART_Packet_t uart7_rx_buffers[UART_RX_BUF_COUNT];

UART_Instance_t uart10_instance;
__attribute__((section(".non_cacheable"))) UART_Packet_t uart10_rx_buffers[UART_RX_BUF_COUNT];

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

static bool start_rx(UART_Instance_t *instance) {
    if (HAL_UARTEx_ReceiveToIdle_DMA(
            instance->uart_handle,
            instance->rx_buffers[instance->active_buffer_index],
            instance->rx_buffer_size) != HAL_OK)
    {
        instance->errors.rx_restart_failures++;
        return false;
    }
 
    // Only idle/transfer-complete events are wanted, not half-transfer
    __HAL_DMA_DISABLE_IT(instance->uart_handle->hdmarx, DMA_IT_HT);
    return true;
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
uint8_t UART_Register(UART_Instance_t *uart_instance, UART_HandleTypeDef *huart, uint8_t *rx_buffer_block, uint16_t rx_buffer_size) {
    if (g_uart_instance_count >= UART_INSTANCE_MAX || uart_instance == NULL || rx_buffer_block == NULL || rx_buffer_size == 0) { // Overflow protection
        return false; 
    }

    // Initialize state
    memset(uart_instance, 0, sizeof(UART_Instance_t));
    uart_instance->uart_handle = huart;
    uart_instance->rx_buffer_size = rx_buffer_size;
    uart_instance->active_buffer_index = 0;
    uart_instance->read_ptr = 0;

    // Split up buffer block into pointer array
    for (int i = 0; i < UART_RX_BUF_COUNT; i++) {
        uart_instance->rx_buffers[i] = rx_buffer_block + (i * sizeof(UART_Packet_t));
    }

    // Create FreeRTOS rx message queue
    uart_instance->msg_queue = xQueueCreate(UART_MSG_QUEUE_SIZE, sizeof(UART_Message_t));
    if (uart_instance->msg_queue == NULL) {
        return false;
    }

    // Create FreeRTOS tx semaphore and mutex
    uart_instance->tx_complete_sem = xSemaphoreCreateBinary();
    if (uart_instance->tx_complete_sem == NULL) {
        vQueueDelete(uart_instance->msg_queue);
        return false;
    }

    uart_instance->tx_mutex = xSemaphoreCreateMutex();
    if (uart_instance->tx_mutex == NULL) {
        vQueueDelete(uart_instance->msg_queue);
        vSemaphoreDelete(uart_instance->tx_complete_sem);
        return false;
    }

    uart_instance->is_initialized = 1;

    if (uart_instance->msg_queue == NULL || uart_instance->tx_complete_sem == NULL) {
        return false;
    }

    // Store the instance in order to iterate through all instances when rx iterrupt is triggered
    g_uart_instances[g_uart_instance_count++] = uart_instance;

    // starts DMA on buffer 0
    if (!start_rx(uart_instance)) {
        g_uart_instance_count--;
        uart_instance->is_initialized = 0;

        vQueueDelete(uart_instance->msg_queue);
        vSemaphoreDelete(uart_instance->tx_complete_sem);
        vSemaphoreDelete(uart_instance->tx_mutex);

        memset(uart_instance, 0, sizeof(UART_Instance_t));
    }

    return true;
}

/**
 * @brief UART receive callback
 * 
 * @param huart UART handle
 * @param Size size of the received data
 * 
 * @note This function is called after DMA_IT_TC (Transfer Complete Interrupt), when
 * the peripheral finishes a transfer. The function creates a message from the rx buffer 
 * that was just filled, then sends it nto the FreeRTOS queue associated with the uart instance.
*/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {

    // Find UART instance
    UART_Instance_t *instance = get_uart_instance(huart);
    if (instance == NULL || !instance->is_initialized) {
        return;
    }

    // Half-Transfer, DMA is still filling buffer
    if (HAL_UARTEx_GetRxEventType(huart) == HAL_UART_RXEVENT_HT) {
        return;
    }

    // Figure out what buffer just finished receiving data
    uint8_t finished_buffer_index = instance->active_buffer_index;

    // Flip index
    instance->active_buffer_index = (instance->active_buffer_index + 1) % UART_RX_BUF_COUNT;

    // Restart DMA into new buffer, keeps reading data while ISR is running
    HAL_UARTEx_ReceiveToIdle_DMA(
        huart, 
        instance->rx_buffers[instance->active_buffer_index], 
        instance->rx_buffer_size
    );
    __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);

    if (Size != instance->rx_buffer_size || xTaskGetSchedulerState() == taskSCHEDULER_NOT_STARTED) {
        return;
    }

    // Create message
    UART_Message_t msg;
    memcpy(msg.payload, instance->rx_buffers[finished_buffer_index], Size);
    msg.length = instance->rx_buffer_size;

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

    // Acquire tx mutex
    if (xSemaphoreTake(instance->tx_mutex, timeout) != pdFALSE) {
        return false;
    }

    // Clear any old semaphore state just in case
    xSemaphoreTake(instance->tx_complete_sem, 0);

    bool status = false;

    // Tell HAL to start the DMA
    if (HAL_UART_Transmit_DMA(instance->uart_handle, tx_buffer, tx_buffer_len) == HAL_OK) {
        if (xSemaphoreTake(instance->tx_complete_sem, timeout) == pdTRUE) {
            status = true;
        }
        else {
            // Condition only true if hardware timed out or froze
            HAL_UART_AbortTransmit(instance->uart_handle);
        }
    }
    else {
        return false;
    }

    xSemaphoreGive(instance->tx_mutex);

    return status;
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

/**
 * @brief UART restart
 *
 * @param instance UART instance
 *
 * @note Used by the daemon to restart reception
*/
uint8_t UART_Restart_Rx(UART_Instance_t *instance)
{
    if (instance == NULL || !instance->is_initialized) {
        return false;
    }
 
    HAL_UART_AbortReceive(instance->uart_handle);
    return start_rx(instance);
}