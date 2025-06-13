#include "supercap.h"

Supercap_t g_supercap;
// CAN_Instance_t *supercap_can_instance;
UART_Instance_t *supercap_uart_instance_ptr;
extern Jetson_Orin_Data_t g_orin_data;
struct rx_data
{
    uint16_t max_discharge;
    uint16_t base_power;
    int16_t cap_energy_percent;
    uint16_t cap_state;
} g_supercap_data;

void Supercap_Init(UART_HandleTypeDef *huartx)
{
    // Initialize supercap
    // g_supercap->can_bus = 1; // can 2
    // g_supercap->tx_id = 0x2C8;
    // g_supercap->rx_id = 0x2C7;
    
    //register uart instance
    supercap_uart_instance_ptr = UART_Register(huartx, g_supercap.rx_buffer, SUPERCAP_RX_BUFFER_SIZE, Supercap_Decode);
}

void Supercap_Decode(UART_Instance_t *supercap_uart_instance_ptr)
{
    // // Recieve supercap data
    // uint16_t *supercap_rx = (uint16_t *) can_instance->rx_buffer;

    // g_supercap_data.max_discharge = supercap_rx[0];
    // g_supercap_data.base_power = supercap_rx[1];
    // g_supercap_data.cap_energy_percent = (int16_t) supercap_rx[2];
    // g_supercap_data.cap_state = supercap_rx[3];
    // // ! do not read more than 8 bytes from the buffer

}

void Supercap_Timeout_Callback(void){
    UART_Service_Init(supercap_uart_instance_ptr);
}

// fomrate PXXXP -  sending XXX as power
// PVONP - Turn on supercap
// PVOFFP - Turn off supercap
void Supercap_Send(void)
{
    // Send supercap data
    // uint16_t *supercap_tx = (uint16_t *) supercap_can_instance->tx_buffer;
    // supercap_tx[0] = 0x003C;
    // supercap_tx[1] = 0x003C;
    // supercap_tx[2] = 0x2012;
    // supercap_tx[3] = 0x0112;
    // ! do not write more than 8 bytes to the buffer
    
    // CAN_Transmit(supercap_can_instance);
    UART_Transmit(supercap_uart_instance_ptr, g_supercap.tx_buffer,sizeof(g_supercap.tx_buffer), UART_DMA);
}