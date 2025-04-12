#include "supercap.h"

#include "referee_system.h"

Supercap_t g_supercap;
CAN_Instance_t *supercap_can_instance;
extern Jetson_Orin_Data_t g_orin_data;

struct rx_data
{
    uint16_t cap_percentage;
    uint16_t reserved1;
    uint16_t reserved2;
    uint16_t reserved3;
} g_supercap_rx_data;

void Supercap_Init(Supercap_t *g_supercap)
{
    // Initialize supercap
    g_supercap->can_bus = 1;
    g_supercap->tx_id = 0x2C8;
    g_supercap->rx_id = 0x2C7;
    supercap_can_instance =
        CAN_Device_Register(g_supercap->can_bus, g_supercap->tx_id,
                            g_supercap->rx_id, Supercap_Decode);
}

void Supercap_Decode(CAN_Instance_t *can_instance)
{
    // Recieve supercap data
    uint16_t *supercap_rx = (uint16_t *) can_instance->rx_buffer;

    g_supercap_rx_data.cap_percentage = supercap_rx[0];
    g_supercap_rx_data.reserved1 = 0;
    g_supercap_rx_data.reserved2 = 0;
    g_supercap_rx_data.reserved3 = 0;
    // ! do not read more than 8 bytes from the buffer
}

void Supercap_Send(void)
{
    // Send supercap data
    uint16_t *supercap_tx = (uint16_t *) supercap_can_instance->tx_buffer;
    supercap_tx[0] = Referee_System.Robot_State.Chassis_Power_Max;    // The power limit from the referee system
    supercap_tx[1] = (uint16_t) Referee_System.Power_Heat.Chassis_Power;        // the total power requested from the chassis
    supercap_tx[2] = 0;
    supercap_tx[3] = 0;
    // ! do not write more than 8 bytes to the buffer
    
    CAN_Transmit(supercap_can_instance);
}