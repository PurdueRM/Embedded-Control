#include "supercap.h"

Supercap_t g_supercap;
CAN_Instance_t *supercap_can_instance;
extern Jetson_Orin_Data_t g_orin_data;
struct rx_data
{
    uint16_t max_discharge;
    uint16_t base_power;
    int16_t cap_energy_percent;
    uint16_t cap_state;
} g_supercap_data;

void Supercap_Init(Supercap_t *g_supercap)
{
    // Initialize supercap
    g_supercap->can_bus = 1; // can 2
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

    g_supercap_data.max_discharge = supercap_rx[0];
    g_supercap_data.base_power = supercap_rx[1];
    g_supercap_data.cap_energy_percent = (int16_t) supercap_rx[2];
    g_supercap_data.cap_state = supercap_rx[3];
    // ! do not read more than 8 bytes from the buffer
}

void Supercap_Send(void)
{
    // Send supercap data
    uint16_t *supercap_tx = (uint16_t *) supercap_can_instance->tx_buffer;
    supercap_tx[0] = 0x003C;
    supercap_tx[1] = 0x003C;
    supercap_tx[2] = 0x2012;
    supercap_tx[3] = 0x0112;
    // ! do not write more than 8 bytes to the buffer
    
    CAN_Transmit(supercap_can_instance);
}