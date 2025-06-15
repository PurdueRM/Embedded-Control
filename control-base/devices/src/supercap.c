#include "supercap.h"
#include "bsp_uart.h"
#include "bsp_daemon.h"
#include "string.h"
#include "stdlib.h"
#include "bsp_serial.h"

#define SUPERCAP_TIMEOUT_MS (3000)
Supercap_t g_supercap;
Daemon_Instance_t *g_supercap_daemon_ptr;
// CAN_Instance_t *supercap_can_instance;
UART_Instance_t *supercap_uart_instance_ptr;
extern Jetson_Orin_Data_t g_orin_data;
char* middle_cpy;
char uart_buffer[SUPERCAP_RX_BUFFER_SIZE];
uint8_t uart_byte;
uint8_t uart_index = 0;
uint8_t supercap_uart_instance_initialized;

// Parsed values
//float Vi = 0, Vo = 0, Pi = 0, Io = 0, Ps = 0, Ii = 0;

struct rx_data
{
    uint16_t max_discharge;
    uint16_t base_power;
    int16_t cap_energy_percent;
    uint16_t cap_state;
} g_supercap_data;


// Function to extract float after a key like "Vi:"
// float extract_value(const char *src, const char *key) {
//     const char *start = strstr(src, key);
//     if (!start) return 0.0f;
//     start += strlen(key);  // move pointer after "Vi:"
//     return atof(start);    // convert to float
// }

void Supercap_Timeout_Callback(void){
    UART_Service_Init(supercap_uart_instance_ptr);
}

void Supercap_Decode_Callback(UART_Instance_t *uart_instance) {
    // static float Vi, Vo, Pi, Io, Ps, Ii;
    
    // uart_instance->rx_buffer[0];
    if (uart_instance->rx_buffer[0] != '\n') {
        
        g_supercap.buffer_for_construction[g_supercap.receive_counter] = uart_instance->rx_buffer[0]; // store the byte in the buffer
        g_supercap.receive_counter++; // R
    }
    else // if the end of frame is reached
    {
        g_supercap.buffer_for_construction[g_supercap.receive_counter] = '\0'; 
        g_supercap.receive_counter = 0; // reset counter if \n is received
        Daemon_Reload(g_supercap_daemon_ptr);

        // Supercap decode logic
        // g_supercap.Vi = extract_value((const char*)g_supercap.buffer_for_construction, "Vi:");
        // g_supercap.Vo = extract_value((const char*)g_supercap.buffer_for_construction, "Vo:");
        // g_supercap.Pi = extract_value((const char*)g_supercap.buffer_for_construction, "Pi:");
        // g_supercap.Ii = extract_value((const char*)g_supercap.buffer_for_construction, "Ii:");
        // g_supercap.Io = extract_value((const char*)g_supercap.buffer_for_construction, "Io:");
        // g_supercap.Ps = extract_value((const char*)g_supercap.buffer_for_construction, "Ps:");
       sscanf((char *)g_supercap.buffer_for_construction, "Vi:%f Vo:%f Pi:%f Ii:%f Io:%f Ps:%f",
              &g_supercap.Vi, &g_supercap.Vo, &g_supercap.Pi, &g_supercap.Ii, &g_supercap.Io, &g_supercap.Ps);
    }
}

void Supercap_Init(UART_HandleTypeDef *huartx)
{
    // Initialize supercap
    // g_supercap->can_bus = 1; // can 2
    // g_supercap->tx_id = 0x2C8;
    // g_supercap->rx_id = 0x2C7;

    //supercap_uart_instance_ptr = UART_Register(huartx, g_supercap.rx_buffer, SUPERCAP_RX_BUFFER_SIZE, Supercap_Decode);
    supercap_uart_instance_ptr = UART_Register(huartx, g_supercap.rx_buffer, SUPERCAP_RX_BUFFER_SIZE, Supercap_Decode_Callback);    // matches expected signature
    uint16_t reload_value = SUPERCAP_TIMEOUT_MS / DAEMON_PERIOD;
    uint16_t intial_counter = reload_value;
    g_supercap_daemon_ptr = Daemon_Register(reload_value, intial_counter, Supercap_Timeout_Callback);
    supercap_uart_instance_initialized = 1; //turn on supercap uart 
}


// fomrate PXXXP -  sending XXX as power
// PVONP - Turn on supercap
// PVOFFP - Turn off supercap
void Supercap_Send(void)
{
     if (!supercap_uart_instance_initialized) {
        return;
    }

    // Send supercap data
    // uint16_t *supercap_tx = (uint16_t *) supercap_can_instance->tx_buffer;
    // supercap_tx[0] = 0x003C;
    // supercap_tx[1] = 0x003C;
    // supercap_tx[2] = 0x2012;
    // supercap_tx[3] = 0x0112;
    // ! do not write more than 8 bytes to the buffer

	// UART_Transmit(supercap_uart_instance_ptr, g_supercap.tx_buffer, sizeof(g_orin_data.tx_buffer), UART_DMA);
    uint8_t max_power = 60;
    DEBUG_PRINTF(supercap_uart_instance_ptr->uart_handle, "P%03dP\r\n", max_power);
}