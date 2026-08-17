#ifndef CAN_H
#define CAN_H

#include "FreeRTOS.h"
#include "queue.h"
#include "stm32h7xx_hal.h"

#define CAN_MAX_DEVICES_PER_BUS 32 // Arbitrarilly set

struct CAN_Instance_t;
struct CAN_Device_t;

// Error Struct
typedef struct CAN_Error_Stats_t {
    // State Machine Faults
    uint32_t bus_off_count;
    uint32_t error_passive_count;
    uint32_t error_warning_count;

    // Protocol / Physics Faults
    uint32_t protocol_error_count;
    uint8_t  last_error_code;       // The raw LEC (Last Error Code)
    
    // Hardware Faults
    uint32_t ram_access_failure_count;

    // Timing
    uint32_t last_error_timestamp;  // FreeRTOS ticks at the time of the last fault
} CAN_Error_Stats_t;

// Struct for each Motor/Device that goes on a can bus
typedef struct CAN_Device_t {
    struct CAN_Instance_t *parent_bus;           // reference to the bus it lives on
    uint16_t rx_id;                         // Hardware filter ID
    FDCAN_TxHeaderTypeDef tx_header;
    uint8_t tx_buffer[8];
    uint8_t rx_buffer[8];
    void (*callback)(struct CAN_Device_t *device);
    void *binding_motor_stats;      /* void pointer to the motor stats
                                    * this is used to bind the motor stats to the can instance
                                    * so that the callback function can decode the can message
                                    */
} CAN_Device_t;

// Struct for each can peripheral
typedef struct CAN_Instance_t {
    FDCAN_HandleTypeDef *hfdcan;            // FDCAN1, FDCAN2, or FDCAN3
    
    CAN_Device_t *registered_devices[CAN_MAX_DEVICES_PER_BUS]; 
    uint8_t device_count;
    uint8_t filter_index;

    CAN_Error_Stats_t errors;
} CAN_Instance_t;

// Message to be sent through queue
typedef struct {
    struct CAN_Instance_t *bus;
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];
} CAN_RxMessage_t;


void CAN_Service_Init();
void CAN_Instance_Init(CAN_Instance_t *bus, FDCAN_HandleTypeDef *hfdcan);
CAN_Device_t *CAN_Device_Register(CAN_Instance_t *bus, uint16_t tx_id, uint16_t rx_id, void (*callback)(CAN_Device_t *));
CAN_Device_t *CAN_Tx_Device_Register(CAN_Instance_t *bus, uint16_t tx_id);
HAL_StatusTypeDef CAN_Transmit(CAN_Device_t *device);
CAN_Instance_t* CAN_Get_Bus_Instance(uint8_t bus_number);
uint8_t BSP_CAN_Get_Bus_Number(CAN_Instance_t *bus_instance);


extern CAN_Instance_t g_can1;
extern CAN_Instance_t g_can2;
extern CAN_Instance_t g_can3;

extern QueueHandle_t CAN_RxQueue;

#endif /* CAN_H */