#include "bsp_can.h"
#include <stdlib.h>

static CAN_Instance_t *g_can1_can_instances[CAN_MAX_DEVICE] = {NULL};
static uint8_t g_can1_device_count = 0;
static CAN_Instance_t *g_can2_can_instances[CAN_MAX_DEVICE] = {NULL};
static uint8_t g_can2_device_count = 0;

CAN_RxHeaderTypeDef g_rx_header;
uint8_t g_can_data[8];

void CAN_Init(CAN_HandleTypeDef *hcanx);
void CAN_SendTOQueue(uint8_t can_bus, uint32_t id, uint8_t data[8]);

/**
 * Initialize the CAN filter for a CAN instance @ref typedef CAN_Instance_t
 * 
 * TODO: utilize the filter bank for better performance, all filter banks are 
 * initialized to filter nothing.
*/
void CAN_Filter_Init(CAN_Instance_t *can_instance)
{
    static uint8_t can1_filter_count = 0, can2_filter_count = 14; // can 1 filter starts from 0, can 2 filter starts from 14
    /* set the CAN filter */
    CAN_FilterTypeDef can_filter;
    // ID mask mode. FilterIdHigh, FilterIdLow, FilterMaskIdHigh, FilterMaskIdLow correspond to registers in the CAN hardware
    // mask mode would define behavior of these registers. Check the reference manual for more information
    can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter.FilterFIFOAssignment = ((can_instance->rx_id & 1) == 0) ? CAN_FILTER_FIFO0 : CAN_FILTER_FIFO1; // match even can id to FIFO0, odd to FIFO1
    can_filter.FilterBank = (can_instance->can_bus == 1) ? can1_filter_count++ : can2_filter_count++;
    can_filter.SlaveStartFilterBank = 14; // CAN 2 is the slave of CAN 1, distribute 0-13 to CAN 1, 14-27 to CAN 2
    can_filter.FilterActivation = CAN_FILTER_ENABLE;
    can_filter.FilterIdHigh = 0; //can_instance->rx_id << 5;                // standard id is 11 bit, so shift 5 bits
    can_filter.FilterIdLow = 0;                 // the second id is not used
    can_filter.FilterMaskIdHigh = 0;            // the third id is not used
    can_filter.FilterMaskIdLow = 0;             // the fourth id is not used
    HAL_CAN_ConfigFilter((can_instance->can_bus == 1) ? &hcan1 : &hcan2, &can_filter);
}
/**
 * @brief  CAN Device Registration Function
 * 
 * @param can_bus can bus number (1 or 2)
 * @param can_id can id of the device (0x000 to 0x7FF)
 * @param can_module_callback callback function for the can module
 * 
 * @return CAN_Instance_t* the pointer to the can_instance
*/
CAN_Instance_t *CAN_Device_Register(uint8_t _can_bus, uint16_t _tx_id, uint16_t _rx_id, void (*can_module_callback)(CAN_Instance_t *can_instance))
{
    CAN_Instance_t *can_instance = malloc(sizeof(CAN_Instance_t));
    
    // define can bus, can id, callback function
    can_instance->can_bus = _can_bus;
    can_instance->rx_id = _rx_id;
    can_instance->can_module_callback = can_module_callback;
    
    // allocate memory for tx_header and rx_header
    can_instance->tx_header = malloc(sizeof(CAN_TxHeaderTypeDef));
    can_instance->tx_header->StdId = _tx_id;
    can_instance->tx_header->IDE = CAN_ID_STD;
    can_instance->tx_header->RTR = CAN_RTR_DATA;
    can_instance->tx_header->DLC = 0x08;

    // assign pointer to the can_instance to the global array
    switch (_can_bus)
    {
    case 1:
        g_can1_can_instances[g_can1_device_count++] = can_instance;
        break;
    case 2:
        g_can2_can_instances[g_can2_device_count++] = can_instance;
        break;
    default:
        // TODO: LOG can bus need to be 1 or 2
        break;
    }
    
    CAN_Filter_Init(can_instance);
    return can_instance;
}

/**
 * @brief  CAN Service Initialization
*/
void CAN_Service_Init()
{
    /* Start CAN Communication */
    HAL_CAN_Start(&hcan1);
    HAL_CAN_Start(&hcan2);

    /* Activate Interrupt */
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO1_MSG_PENDING);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING);
}

/**
 * @brief  Transmit a CAN message
*/
HAL_StatusTypeDef CAN_Transmit(CAN_Instance_t *can_instance)
{
    // Select the correct CAN bus
    CAN_HandleTypeDef *hcanx = (can_instance->can_bus == 1) ? &hcan1 : &hcan2;
    // Wait for available mailbox
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcanx) == 0);
    // Transmit the message
    return HAL_CAN_AddTxMessage(hcanx, can_instance->tx_header, can_instance->tx_buffer, (uint32_t *)CAN_TX_MAILBOX0);
}

/**
 * @brief  Callback function for CAN Receive. This function is called when a message
 * is received in FIFO0 or FIFO1. The function will iterate through the registered
 * can instances and call the callback function binded to the can instance
*/
void CAN_Rx_Callback(CAN_HandleTypeDef *hcan, uint8_t fifo_num) {
    static CAN_RxHeaderTypeDef rx_header;
    uint8_t can_rx_buff[8];
    // Get the message from the FIFO
    if (HAL_CAN_GetRxMessage(hcan, fifo_num, &rx_header, can_rx_buff) == HAL_OK)
    {
        // Select the correct CAN bus
        uint8_t can_bus = (hcan->Instance == hcan1.Instance) ? 1 : 2;
        switch (can_bus)
        {
        case 1:
            // Iterate through the registered can1 instances
            for (uint8_t i = 0; i < g_can1_device_count; i++)
            {
                if (g_can1_can_instances[i]->rx_id == rx_header.StdId)
                {
                    // move the received data to the rx buffer in the can instance
                    memmove(g_can1_can_instances[i]->rx_buffer, can_rx_buff, 8);
                    g_can1_can_instances[i]->can_module_callback(g_can1_can_instances[i]);
                    break;
                }
            }
            break;
        case 2:
            // Iterate through the registered can1 instances
            for (uint8_t i = 0; i < g_can2_device_count; i++)
            {
                if (g_can2_can_instances[i]->rx_id == rx_header.StdId)
                {
                    // move the received data to the rx buffer in the can instance
                    memmove(g_can2_can_instances[i]->rx_buffer, can_rx_buff, 8);
                    g_can2_can_instances[i]->can_module_callback(g_can2_can_instances[i]);
                    break;
                }
            }
            break;
        default:
        // TODO: LOG can bus need to be 1 or 2
            break;
        }
    }
}

/**
 * @brief  CAN FIFO0 Receive Callback. This function is called when a message 
 * is received in FIFO0. For better abstraction, user defined CAN_Rx_Callback()
 * is called to decode the message with the function pointer binded with each
 * can instance
 * 
 * this function overrides the weak implementation in stm32f4xx_hal_can.c
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_Rx_Callback(hcan, CAN_RX_FIFO0);
}

/**
 * @brief  CAN FIFO1 Receive Callback. This function is called when a message
 * is received in FIFO1. For better abstraction, user defined CAN_Rx_Callback()
 * is called to decode the message with the function pointer binded with each
 * can instance
 * 
 * this function overrides the weak implementation in stm32f4xx_hal_can.c
*/
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_Rx_Callback(hcan, CAN_RX_FIFO1);
}
