#ifndef CAN_H
#define CAN_H

#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "stm32h7xx_hal.h"

#define CAN_MAX_DEVICES_PER_BUS 32 // Arbitrarilly set

struct CAN_Instance_t;
struct CAN_Device_t;
enum CAN_State_t;

typedef enum
{
    CAN_STATE_STOPPED = 0, // peripheral is not running
    CAN_STATE_NORMAL, // peripheral is operating normally
    CAN_STATE_ERROR_WARNING, // TEC or REC has reached 96
    CAN_STATE_ERROR_PASSIVE, // TEC or REC has reached 128
    CAN_STATE_BUS_OFF, // TEC or REC has reached 255, must restart peripheral
    CAN_STATE_RESTARTING // restarting peripheral
} CAN_State_t;

// PSR Decoding

typedef enum
{
    CAN_LEC_NO_ERROR = 0, // no error detected
    CAN_LEC_STUFF_ERROR, // a bit-stuffing rule was violated (look it up)
    CAN_LEC_FORM_ERROR, // a fixed-format field contained an invalid bit value
    CAN_LEC_ACK_ERROR, // transmitter did not receive acknowledgment from another node
    CAN_LEC_BIT1_ERROR, // controller transmitted a recessive bit but observed a dominant bit
    CAN_LEC_BIT0_ERROR, // controller transmitted a dominant bit but observed a recessive bit
    CAN_LEC_CRC_ERROR,  // the received frame failed the crc check
    CAN_LEC_NO_CHANGE // no new error code has been recorded since the last psr read or init
} CAN_LastError_t;

typedef enum
{
    CAN_ACTIVITY_SYNCHRONIZING = 0, // waiting to synchronize with can bus
    CAN_ACTIVITY_IDLE, // not currently transmitting or receiving
    CAN_ACTIVITY_RECEIVING, // currently receiving
    CAN_ACTIVITY_TRANSMITTING // currently transmitting
} CAN_Activity_t;

typedef enum
{
    CAN_ERROR_WARNING_INACTIVE = 0, // not in error warning state (TEC or REC < 96)
    CAN_ERROR_WARNING_ACTIVE // in error warning state (TEC or REC >= 96)
} CAN_ErrorWarning_t;

typedef enum
{
    CAN_ERROR_ACTIVE = 0, // when error is detected, the peripheral signals active error to every node on the bus
    CAN_ERROR_PASSIVE // there were too many errors (TEC or REC >= 128), so signaling passive errors and suspends transmission after every transmit
} CAN_ErrorPassive_t;

typedef enum
{
    CAN_BUS_OFF_INACTIVE = 0, // bus is still on (TEC or REC < 255)
    CAN_BUS_OFF_ACTIVE // bus saw too many errors and must restart (TEC or REC >= 255)
} CAN_BusOff_t;

typedef struct
{
    CAN_LastError_t lec; // last error code
    CAN_Activity_t activity;
    CAN_ErrorPassive_t error_passive;
    CAN_ErrorWarning_t error_warning;
    CAN_BusOff_t bus_off;
    CAN_LastError_t dlec; // change in error code

} CAN_PSR_Status_t;

// Error Struct
typedef struct CAN_Error_Stats_t {
    // CAN Protocol State
    uint32_t error_warning_count; // TEC (transmit error count) or REC (receive error count) reach 96 
    uint32_t error_passive_count; // TEC or REC reaches 128, 
    uint32_t bus_off_count; // TEC or REC reaches 255, need to reset bus
    
    // Protocol Errors
    uint32_t arbitration_error_count; // Protocol Error in Arbitration Phase (Nominal Bit Time is used)
    uint32_t data_error_count; // Protocol Error in Data Phase (Data Bit Time is used)

    // Peripheral Errors
    uint32_t peripheral_error_count;
    uint32_t ram_access_error_count;
    uint32_t ram_watchdog_error_count;
    uint32_t reserved_area_error_count;
    uint32_t error_log_overflow_count;
    uint32_t timeout_count;

    // Last error diagnostics
    uint32_t last_error_code; // raw lec
    uint32_t last_psr; // raw psr (protocol status register)
    uint32_t last_ecr; // raw ecr (error counter register)

    uint8_t last_tec; // transmission error count
    uint8_t last_rec; // receiving error count

    uint32_t last_hal_error;

    // Timing
    uint32_t last_error_timestamp;  // FreeRTOS ticks at the time of the last fault

    CAN_PSR_Status_t decoded_psr;
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

    SemaphoreHandle_t tx_mutex;

    CAN_State_t state;
    CAN_Error_Stats_t errors;
} CAN_Instance_t;

// Message to be sent through queue
typedef struct {
    struct CAN_Instance_t *bus;
    FDCAN_RxHeaderTypeDef header;
    uint8_t data[8];
} CAN_RxMessage_t;


void CAN_Service_Init();
HAL_StatusTypeDef CAN_Service_Start();
void CAN_Instance_Init(CAN_Instance_t *bus, FDCAN_HandleTypeDef *hfdcan);
CAN_Device_t *CAN_Device_Register(CAN_Instance_t *bus, uint16_t tx_id, uint16_t rx_id, void (*callback)(CAN_Device_t *));
CAN_Device_t *CAN_Tx_Device_Register(CAN_Instance_t *bus, uint16_t tx_id);
HAL_StatusTypeDef CAN_Transmit(CAN_Device_t *device);
CAN_Instance_t* CAN_Get_Bus_Instance(uint8_t bus_number);
void CAN_Begin_Restart(CAN_Instance_t *bus);
void CAN_Poll_Restart(CAN_Instance_t *bus);

uint8_t BSP_CAN_Get_Bus_Number(CAN_Instance_t *bus_instance);
void CAN_CaptureDiagnostics(CAN_Instance_t *bus, FDCAN_HandleTypeDef *hfdcan);


extern CAN_Instance_t g_can1;
extern CAN_Instance_t g_can2;
extern CAN_Instance_t g_can3;

extern QueueHandle_t CAN_RxQueue;

#endif /* CAN_H */