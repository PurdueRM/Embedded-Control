#ifndef CAN_H
#define CAN_H

#include "FreeRTOS.h"
#include "can.h"
#include "queue.h"
#include "cmsis_os.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define CAN_MAX_DEVICE (8)

typedef struct _
{
    uint8_t can_bus;
    CAN_TxHeaderTypeDef *tx_header;
    uint16_t rx_id;
    uint8_t tx_buffer[8];
    uint8_t rx_buffer[8];
    void (*can_module_callback)(struct _ *);
    void *binding_motor_stats;      /* void pointer to the motor stats
                                    * this is used to bind the motor stats to the can instance
                                    * so that the callback function can decode the can message
                                    */
} CAN_Instance_t;

/*
 * Init the filter and start CAN communication 
 */
void CAN_Init(CAN_HandleTypeDef *hcanx);

/*
 * Send the can message to Message Queue
 */
void CAN_SendTOQueue(uint8_t can_bus, uint32_t id, uint8_t data[8]);

/*
 * Send the can message
 */
void CAN_Send(CAN_HandleTypeDef *hcanx, uint32_t id, uint8_t data[8]);


CAN_Instance_t *CAN_Device_Register(uint8_t can_bus, uint16_t tx_id, uint16_t rx_id, void (*can_module_callback)(CAN_Instance_t *can_instance));
void CAN_Service_Init(void);
HAL_StatusTypeDef CAN_Transmit(CAN_Instance_t *can_instance);
#endif
