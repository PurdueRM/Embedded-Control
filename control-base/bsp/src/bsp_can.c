#include "bsp_can.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "stm32h7xx_hal_fdcan.h"
#include <string.h>

#define TOTAL_MAX_DEVICES (CAN_MAX_DEVICES_PER_BUS * 3)
#define CAN_RX_QUEUE_SIZE 128

static CAN_Device_t g_device_pool[TOTAL_MAX_DEVICES]; // Statically allocated CAN device pool
static uint16_t g_device_allocated_count = 0;
CAN_Instance_t *g_buses[3] = {NULL, NULL, NULL};

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

CAN_Instance_t g_can1;
CAN_Instance_t g_can2;
CAN_Instance_t g_can3;
QueueHandle_t CAN_RxQueue = NULL;

extern TaskHandle_t can_restart_task_handle;

void CAN_Service_Init() {
    CAN_RxQueue = xQueueCreate(CAN_RX_QUEUE_SIZE, sizeof(CAN_RxMessage_t));

    CAN_Instance_Init(&g_can1, &hfdcan1);
    CAN_Instance_Init(&g_can2, &hfdcan2);
    CAN_Instance_Init(&g_can3, &hfdcan3);
}

/**
 * @brief Initialize the Bus Object, Queue, and Hardware
 */
void CAN_Instance_Init(CAN_Instance_t *bus, FDCAN_HandleTypeDef *hfdcan) {
    bus->hfdcan = hfdcan;
    bus->device_count = 0;
    bus->filter_index = 0;

    if (hfdcan == &hfdcan1) {
        g_buses[0] = bus;
    }
    else if (hfdcan == &hfdcan2) {
        g_buses[1] = bus;
    }
    else if (hfdcan == &hfdcan3) {
        g_buses[2] = bus;
    }
    else {
        return;
    }

    // Set error struct to 0
    memset(&bus->errors, 0, sizeof(CAN_Error_Stats_t));

    // Init tx semaphore
    bus->tx_mutex = xSemaphoreCreateMutex();
    configASSERT(bus->tx_mutex != NULL);

    // Reject non-matching frames
    HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_REJECT, FDCAN_REJECT, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    // Route Rx Data to Line 0
    HAL_FDCAN_ConfigInterruptLines(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_INTERRUPT_LINE0);

    // Route Critical Errors to Line 1
    uint32_t error_flags =
    // State Machine Errors (Bus degradation)
        FDCAN_IT_BUS_OFF | // Must restart peripheral, too many errors (TEC or REC == 255) 
        FDCAN_IT_ERROR_PASSIVE | // Detected degrading communincation, stops holding bus with errors (TEC or REC >= 128)
        FDCAN_IT_ERROR_WARNING | // Early warning of accummulated errors (TEC or REC  >= 96)

    // Protocol Errors (The actual physics/logic of the bus failing)
        FDCAN_IT_ARB_PROTOCOL_ERROR |   // Stuff error, CRC error, Form error, etc.
        FDCAN_IT_DATA_PROTOCOL_ERROR |  // (Mostly for FD, but good to leave on)

    // Hardware Errors (Internal STM32 RAM / Clocking issues)
        FDCAN_IT_RAM_ACCESS_FAILURE |   // Message RAM is unpowered or misconfigured
        FDCAN_IT_RAM_WATCHDOG | // Message RAM watchdog events failed

        FDCAN_IT_ERROR_LOGGING_OVERFLOW | // Detected lost error log information
        FDCAN_IT_RESERVED_ADDRESS_ACCESS; // Detected invalid Message RAM/register access

    HAL_FDCAN_ConfigInterruptLines(hfdcan, error_flags, FDCAN_INTERRUPT_LINE1);

    FDCAN_FilterTypeDef sFilterConfig;
    sFilterConfig.IdType = FDCAN_STANDARD_ID;
    sFilterConfig.FilterIndex = 0;
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    sFilterConfig.FilterID1 = 0x000;
    sFilterConfig.FilterID2 = 0x000; // Mask of 0 accepts everything

    if (HAL_FDCAN_ConfigFilter(hfdcan, &sFilterConfig) != HAL_OK)
    {
        Error_Handler();
    }

    // Enable the sources
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_FDCAN_ActivateNotification(hfdcan, error_flags, 0) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {
        Error_Handler();
    }
    bus->state = CAN_STATE_NORMAL;
}

CAN_Device_t *CAN_Device_Register(CAN_Instance_t *bus, uint16_t tx_id, uint16_t rx_id, void (*callback)(CAN_Device_t *)) {

    // Overflow protection
    if (bus->device_count >= CAN_MAX_DEVICES_PER_BUS || g_device_allocated_count >= TOTAL_MAX_DEVICES) {
        return NULL;
    }

    CAN_Device_t *device = &g_device_pool[g_device_allocated_count++];

    device->parent_bus = bus;
    device->rx_id = rx_id;
    device->callback = callback;

    // Init device tx header
    device->tx_header.Identifier = tx_id;
    device->tx_header.IdType = FDCAN_STANDARD_ID;
    device->tx_header.TxFrameType = FDCAN_DATA_FRAME;
    device->tx_header.DataLength = FDCAN_DLC_BYTES_8;
    device->tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    device->tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    device->tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    device->tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    device->tx_header.MessageMarker = 0;

    // Attatch device to bus
    bus->registered_devices[bus->device_count++] = device;

    return device;
}

// Used to register devices which only use tx (like dji motor send groups)
CAN_Device_t *CAN_Tx_Device_Register(CAN_Instance_t *bus, uint16_t tx_id) {
    // Overflow protection
    if (bus->device_count >= CAN_MAX_DEVICES_PER_BUS || g_device_allocated_count >= TOTAL_MAX_DEVICES) {
        return NULL;
    }

    CAN_Device_t *device = &g_device_pool[g_device_allocated_count++];

    device->parent_bus = bus;
    device->rx_id = 0;       // Unused for tx-only
    device->callback = NULL; // Unused for tx-only

    // Init device tx header
    device->tx_header.Identifier = tx_id;
    device->tx_header.IdType = FDCAN_STANDARD_ID;
    device->tx_header.TxFrameType = FDCAN_DATA_FRAME;
    device->tx_header.DataLength = FDCAN_DLC_BYTES_8;
    device->tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    device->tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    device->tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    device->tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    device->tx_header.MessageMarker = 0;

    // Attach device to bus
    bus->registered_devices[bus->device_count++] = device;

    return device;
}

HAL_StatusTypeDef CAN_Transmit(CAN_Device_t *device)
{
    if (device == NULL || device->parent_bus == NULL) {
        return HAL_ERROR;
    }

    CAN_Instance_t *bus = device->parent_bus;
    FDCAN_HandleTypeDef *hfdcan = device->parent_bus->hfdcan;

    if (bus->state != CAN_STATE_NORMAL) {
        return HAL_BUSY;
    }

    // Wait up to 1 ms for bus access (prevents priority inversion)
    if (xSemaphoreTake(device->parent_bus->tx_mutex, pdMS_TO_TICKS(1)) != pdTRUE) {
        return HAL_BUSY;
    }

    HAL_StatusTypeDef status = HAL_BUSY;

    if (bus->state == CAN_STATE_NORMAL) {
        if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0) {
            status = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &device->tx_header, device->tx_buffer);
        }
    }

    xSemaphoreGive(device->parent_bus->tx_mutex);
    return status;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
        CAN_RxMessage_t rxMsg;
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        // Look up the correct bus
        CAN_Instance_t *target_bus = NULL;
        if (hfdcan->Instance == FDCAN1) {
            target_bus = g_buses[0];
        }
        else if (hfdcan->Instance == FDCAN2) {
            target_bus = g_buses[1];
        }
        else if (hfdcan->Instance == FDCAN3) {
            target_bus = g_buses[2];
        }
        else {
            return;
        }

        while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0) {
            HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxMsg.header, rxMsg.data);

            // Push to that bus's queue
            if (target_bus != NULL && CAN_RxQueue != NULL) {
                rxMsg.bus = target_bus;
                xQueueSendFromISR(CAN_RxQueue, &rxMsg, &xHigherPriorityTaskWoken); // handle what to do if queue is full
            }
        }

        // Force a context switch if a higher priority task was unblocked
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

/*
 * Error Handling
 */

void HAL_FDCAN_ErrorStatusCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t ErrorStatusITs) {
    CAN_Instance_t *bus;

    if (hfdcan->Instance == FDCAN1) {
        bus = g_buses[0];
    }
    else if (hfdcan->Instance == FDCAN2) {
        bus = g_buses[1];
    }
    else if (hfdcan->Instance == FDCAN1) {
        bus = g_buses[2];
    }
    else {
        return;
    }

    CAN_CaptureDiagnostics(bus, hfdcan);

    if (ErrorStatusITs & FDCAN_IT_ERROR_WARNING) { // Error warning
        bus->errors.error_warning_count++;
    }

    if (ErrorStatusITs & FDCAN_IT_ERROR_PASSIVE) { // Error passive
        bus->errors.error_passive_count++;
    }

    if (ErrorStatusITs & FDCAN_IT_BUS_OFF) { // Bus off
        bus->errors.bus_off_count++;
        bus->state = CAN_STATE_BUS_OFF;

        if (can_restart_task_handle != NULL) {
            BaseType_t higher_priority_task_woken = pdFALSE;

            vTaskNotifyGiveFromISR( // notify revocery task
                can_restart_task_handle,
                &higher_priority_task_woken
            );

            portYIELD_FROM_ISR(higher_priority_task_woken);
        }
    }

    // Update state
    uint32_t psr = hfdcan->Instance->PSR;

    if (psr & FDCAN_PSR_BO)
    {
        bus->state = CAN_STATE_BUS_OFF;
    }
    else if (psr & FDCAN_PSR_EP)
    {
        bus->state = CAN_STATE_ERROR_PASSIVE;
    }
    else if (psr & FDCAN_PSR_EW)
    {
        bus->state = CAN_STATE_ERROR_WARNING;
    }
    else if (bus->state != CAN_STATE_RESTARTING)
    {
        bus->state = CAN_STATE_NORMAL;
    }
}

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan) { // Peripheral error handling
    CAN_Instance_t *bus;

    if (hfdcan->Instance == FDCAN1) {
        bus = g_buses[0];
    }
    else if (hfdcan->Instance == FDCAN2) {
        bus = g_buses[1];
    }
    else if (hfdcan->Instance == FDCAN1) {
        bus = g_buses[2];
    }
    else {
        return;
    }

    CAN_CaptureDiagnostics(bus, hfdcan);

    // Hal errors are sticky, once you access them they clear
    uint32_t error = HAL_FDCAN_GetError(hfdcan);

    bus->errors.last_hal_error = error;

    if (error & HAL_FDCAN_ERROR_RAM_ACCESS)
        bus->errors.ram_access_error_count++;

    if (error & HAL_FDCAN_ERROR_RAM_WDG)
        bus->errors.ram_watchdog_error_count++;

    if (error & HAL_FDCAN_ERROR_PROTOCOL_ARBT)
        bus->errors.arbitration_error_count++;

    if (error & HAL_FDCAN_ERROR_PROTOCOL_DATA)
        bus->errors.data_error_count++;

    if (error & HAL_FDCAN_ERROR_RESERVED_AREA)
        bus->errors.reserved_area_error_count++;

    if (error & HAL_FDCAN_ERROR_LOG_OVERFLOW)
        bus->errors.error_log_overflow_count++;

    if (error & HAL_FDCAN_ERROR_TIMEOUT)
        bus->errors.timeout_count++;

    bus->errors.peripheral_error_count++;
}

// Called from CAN restart task
void CAN_Begin_Restart(CAN_Instance_t *bus) {
    if (bus == NULL || bus->state != CAN_STATE_BUS_OFF) {
        return;
    }

    FDCAN_HandleTypeDef *hfdcan = bus->hfdcan;

    bus->state = CAN_STATE_RESTARTING;

    CLEAR_BIT(hfdcan->Instance->CCCR, FDCAN_CCCR_INIT);
}

// Periodically called in CAN task, returns driver to NORMAL state after recovery completes
void CAN_Poll_Restart(CAN_Instance_t *bus) {
    if (bus == NULL || bus->state != CAN_STATE_RESTARTING) {
        return;
    }

    uint32_t psr = bus->hfdcan->Instance->PSR;

    if (psr & FDCAN_PSR_BO) {
        return;
    }

    if (psr & FDCAN_PSR_EP) {
        bus->state = CAN_STATE_ERROR_PASSIVE;
    }
    else if (psr & FDCAN_PSR_EW) {
        bus->state = CAN_STATE_ERROR_WARNING;
    }
    else {
        bus->state = CAN_STATE_NORMAL;
    }
}

/* 
 * Helpers
 */

CAN_Instance_t* CAN_Get_Bus_Instance(uint8_t bus_number) {
    switch (bus_number) {
        case 1:
            return &g_can1;
        case 2:
            return &g_can2;
        case 3:
            return &g_can3;
        default:
            return NULL; // Invalid bus number
    }
}

uint8_t BSP_CAN_Get_Bus_Number(CAN_Instance_t *bus_instance) {
    // We compare the memory addresses of the pointers
    if (bus_instance == &g_can1) {
        return 1;
    } 
    else if (bus_instance == &g_can2) {
        return 2;
    } 
    else if (bus_instance == &g_can3) {
        return 3;
    } 
    
    return 0; // Invalid pointer
}

void CAN_CaptureDiagnostics(CAN_Instance_t *bus, FDCAN_HandleTypeDef *hfdcan) {
    uint32_t psr = hfdcan->Instance->PSR;

    CAN_PSR_Status_t * status = &bus->errors.decoded_psr;

    status->lec = (CAN_LastError_t)
        ((psr & FDCAN_PSR_LEC) >> FDCAN_PSR_LEC_Pos);

    status->activity = (CAN_Activity_t)
        ((psr & FDCAN_PSR_ACT) >> FDCAN_PSR_ACT_Pos);

    status->error_passive = (CAN_ErrorPassive_t)
        ((psr & FDCAN_PSR_EP) >> FDCAN_PSR_EP_Pos);

    status->error_warning = (CAN_ErrorWarning_t)
        ((psr & FDCAN_PSR_EW) >> FDCAN_PSR_EW_Pos);

    status->bus_off = (CAN_BusOff_t)
        ((psr & FDCAN_PSR_BO) >> FDCAN_PSR_BO_Pos);

    status->dlec = (CAN_LastError_t)
        ((psr & FDCAN_PSR_DLEC) >> FDCAN_PSR_DLEC_Pos);


    uint32_t ecr = hfdcan->Instance->ECR;

    bus->errors.last_psr = psr;
    bus->errors.last_ecr = ecr;

    bus->errors.last_error_code = psr & FDCAN_PSR_LEC;

    bus->errors.last_tec = (uint8_t)((ecr & FDCAN_ECR_TEC) >> FDCAN_ECR_TEC_Pos);

    bus->errors.last_rec = (uint8_t)((ecr & FDCAN_ECR_REC) >> FDCAN_ECR_REC_Pos);

    bus->errors.last_error_timestamp = xTaskGetTickCountFromISR();
}
