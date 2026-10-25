#include "bsp_can.h"
#include "main.h"
#include "stm32h7xx_hal_fdcan.h"
#include <string.h>

#define TOTAL_MAX_DEVICES (CAN_MAX_DEVICES_PER_BUS * 3)
#define CAN_RX_QUEUE_SIZE 128

static CAN_Device_t g_device_pool[TOTAL_MAX_DEVICES]; // Statically allocated CAN device pool
static uint16_t g_device_allocated_count = 0;
static CAN_Instance_t *g_buses[3] = {NULL, NULL, NULL};

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

CAN_Instance_t g_can1;
CAN_Instance_t g_can2;
CAN_Instance_t g_can3;
QueueHandle_t CAN_RxQueue = NULL;


HAL_StatusTypeDef CAN_Service_Start() {
    HAL_StatusTypeDef status;
    for (int i = 0; i < 3; i++) {
        FDCAN_HandleTypeDef *hfdcan = g_buses[i]->hfdcan;
        uint32_t error_flags = 
            FDCAN_IT_BUS_OFF | 
            FDCAN_IT_ERROR_PASSIVE | 
            FDCAN_IT_ERROR_WARNING |
            FDCAN_IT_ARB_PROTOCOL_ERROR |  
            FDCAN_IT_DATA_PROTOCOL_ERROR |
            FDCAN_IT_RAM_ACCESS_FAILURE;   // Message RAM is unpowered or misconfigured
        status =  HAL_FDCAN_ActivateNotification(hfdcan, error_flags, 0);

        if (status != HAL_OK) {
            Error_Handler();
        }

        status = HAL_FDCAN_Start(hfdcan);

        if (status != HAL_OK) {
            Error_Handler();
        }

        uint32_t cccr = hfdcan1.Instance->CCCR;
        uint32_t state = hfdcan1.State;
        uint32_t error = hfdcan1.ErrorCode;
        uint32_t txfqs = hfdcan1.Instance->TXFQS;

         if (cccr == state || error == txfqs) {
            Error_Handler();
        }
    }
    return status;
}

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
        // hfdcan->Instance = FDCAN1;
        // hfdcan->Init.MessageRAMOffset = 0; // fix this 
    }
    else if (hfdcan == &hfdcan2) {
        g_buses[1] = bus;
        // hfdcan->Instance = FDCAN2;
        // hfdcan->Init.MessageRAMOffset = 512; // fix this 
    }
    else if (hfdcan == &hfdcan3) {
        g_buses[2] = bus;
        // hfdcan->Instance = FDCAN3;
        // hfdcan->Init.MessageRAMOffset = 0; // fdcan3 uses separate ram block
    }
    else {
        return;
    }

    // Set error struct to 0
    memset(&bus->errors, 0, sizeof(CAN_Error_Stats_t));

    // Init tx semaphore
    bus->tx_mutex = xSemaphoreCreateMutex();
    configASSERT(bus->tx_mutex != NULL);

    // HAL_FDCAN_DeInit(hfdcan);

    // Hardware init
    // hfdcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;
    // hfdcan->Init.Mode = FDCAN_MODE_NORMAL;
    // hfdcan->Init.AutoRetransmission = ENABLE;
    // hfdcan->Init.TransmitPause = DISABLE;
    // hfdcan->Init.ProtocolException = DISABLE;

    // hfdcan->Init.NominalPrescaler = 6;
    // hfdcan->Init.NominalSyncJumpWidth = 2;
    // hfdcan->Init.NominalTimeSeg1 = 15;
    // hfdcan->Init.NominalTimeSeg2 = 4;
    // hfdcan->Init.DataPrescaler = 1;

    // hfdcan->Init.DataSyncJumpWidth = 1;
    // hfdcan->Init.DataTimeSeg1 = 1;
    // hfdcan->Init.DataTimeSeg2 = 1;
    // hfdcan->Init.MessageRAMOffset = 0;
    // hfdcan->Init.StdFiltersNbr = 0;
    // hfdcan->Init.ExtFiltersNbr = 0;
    // hfdcan->Init.RxFifo0ElmtsNbr = 16;
    // hfdcan->Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;
    // hfdcan->Init.RxFifo1ElmtsNbr = 0;
    // hfdcan->Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;
    // hfdcan->Init.RxBuffersNbr = 0;
    // hfdcan->Init.RxBufferSize = FDCAN_DATA_BYTES_8;
    // hfdcan->Init.TxEventsNbr = 0;
    // hfdcan->Init.TxBuffersNbr = 0;
    // hfdcan->Init.TxFifoQueueElmtsNbr = 16;
    // hfdcan->Init.TxFifoQueueMode = FDCAN_TX_QUEUE_OPERATION;
    // hfdcan->Init.TxElmtSize = FDCAN_DATA_BYTES_8;

    // if (HAL_FDCAN_Init(hfdcan) != HAL_OK) {
    //     __disable_irq();
    //     while (1) {
    //         ;
    //     }
    // }

    // Reject non-matching frames
    HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_REJECT_REMOTE, FDCAN_REJECT_REMOTE);

    // Route Rx Data to Line 0
    HAL_FDCAN_ConfigInterruptLines(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, FDCAN_INTERRUPT_LINE0);
    
    // Route Critical Errors to Line 1
    uint32_t error_flags = 
    // State Machine Errors (Bus degradation)
        FDCAN_IT_BUS_OFF | 
        FDCAN_IT_ERROR_PASSIVE | 
        FDCAN_IT_ERROR_WARNING |
    
    // Protocol Errors (The actual physics/logic of the bus failing)
        FDCAN_IT_ARB_PROTOCOL_ERROR |   // Stuff error, CRC error, Form error, etc.
        FDCAN_IT_DATA_PROTOCOL_ERROR |  // (Mostly for FD, but good to leave on)
    
    // Hardware Errors (Internal STM32 RAM / Clocking issues)
        FDCAN_IT_RAM_ACCESS_FAILURE;   // Message RAM is unpowered or misconfigured
    
    HAL_FDCAN_ConfigInterruptLines(hfdcan, error_flags, FDCAN_INTERRUPT_LINE1);
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

    // Program the hardware filter
    // FDCAN_FilterTypeDef sFilterConfig;
    // sFilterConfig.IdType = FDCAN_STANDARD_ID;
    // sFilterConfig.FilterIndex = bus->filter_index++;
    // sFilterConfig.FilterType = FDCAN_FILTER_DUAL;
    // sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    // sFilterConfig.FilterID1 = rx_id;
    // sFilterConfig.FilterID2 = rx_id;
    // HAL_FDCAN_ConfigFilter(bus->hfdcan, &sFilterConfig);

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
    FDCAN_HandleTypeDef *hfdcan = device->parent_bus->hfdcan;

    // Wait up to 1 ms for bus access (prevents priority inversion)
    if (xSemaphoreTake(device->parent_bus->tx_mutex, pdMS_TO_TICKS(1)) != pdTRUE) {
        return HAL_BUSY;
    }

    HAL_StatusTypeDef status = HAL_BUSY;

    uint32_t fdcan_sel =
    (RCC->D2CCIP1R & RCC_D2CCIP1R_FDCANSEL) >> 28;

    if (HAL_FDCAN_GetTxFifoFreeLevel(hfdcan) > 0 || !fdcan_sel) {
        status = HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &device->tx_header, device->tx_buffer);
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

void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
    // Look up which bus threw the error
    CAN_Instance_t *target_bus = NULL;
    extern CAN_Instance_t *g_buses[3];

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

    // Get the raw hardware error flags
    uint32_t error_status = hfdcan->Instance->IR;
    
    // Record the time of the failure
    target_bus->errors.last_error_timestamp = xTaskGetTickCountFromISR();

    // State machine errors
    if (error_status & FDCAN_IR_BO) {
        target_bus->errors.bus_off_count++;
        __HAL_FDCAN_CLEAR_FLAG(hfdcan, FDCAN_FLAG_BUS_OFF);
    }
    
    if (error_status & FDCAN_IR_EP) {
        target_bus->errors.error_passive_count++;
        __HAL_FDCAN_CLEAR_FLAG(hfdcan, FDCAN_FLAG_ERROR_PASSIVE);
    }
    
    if (error_status & FDCAN_IR_EW) {
        target_bus->errors.error_warning_count++;
        __HAL_FDCAN_CLEAR_FLAG(hfdcan, FDCAN_FLAG_ERROR_WARNING);
    }

    // Protocol / Physics Errors
    if (error_status & (FDCAN_IR_PEA | FDCAN_IR_PED)) {
        target_bus->errors.protocol_error_count++;
        
        // Extract the Last Error Code (LEC) from the Protocol Status Register
        target_bus->errors.last_error_code = (hfdcan->Instance->PSR & FDCAN_PSR_LEC);

        __HAL_FDCAN_CLEAR_FLAG(hfdcan, FDCAN_FLAG_ARB_PROTOCOL_ERROR | FDCAN_FLAG_DATA_PROTOCOL_ERROR);
    }

    // Hardware Errors
    if (error_status & FDCAN_IR_MRAF) {
        target_bus->errors.ram_access_failure_count++;
        __HAL_FDCAN_CLEAR_FLAG(hfdcan, FDCAN_FLAG_RAM_ACCESS_FAILURE);
    }
}

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