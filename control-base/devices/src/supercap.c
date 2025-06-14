#include "supercap.h"
#include "bsp_uart.h"
#include "bsp_daemon.h"
#include "string.h"
#include "stdlib.h"

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

// // Recieve supercap data
    // uint16_t *supercap_rx = (uint16_t *) can_instance->rx_buffer;
  
void Supercap_Decode(UART_Instance_t *uart_instance, float *Vi, float *Vo, float *Pi, float *Ii, float *Io, float *Ps) 
{
    // uint8_t uart_byte;
    // static char uart_buffer[SUPERCAP_RX_BUFFER_SIZE];
    // static uint16_t uart_index = 0;
    // sscanf((char *)uart_instance->rx_buffer, "Vi:%2.2f Vo:%2.2f Pi:%3.2f Ii:%2.2f Io:%2.2f Ps:%3.2f\r\n",
    //                Vi, Vo, Pi, Ii, Io, Ps);
    // while (1) {
    //     HAL_UART_Receive(uart_instance->uart_handle, &uart_byte, 1, HAL_MAX_DELAY);

    //     if (uart_byte == '\n') {
    //         uart_buffer[uart_index] = '\0';  // Null-terminate the string

    //         // Copy the buffer to rx_buffer if needed
    //         strncpy((char *)uart_instance->rx_buffer, uart_buffer, uart_index + 1);

    //         // Attempt to parse the expected format
    //         sscanf((char *)uart_instance->rx_buffer, "Vi:%f Vo:%f Pi:%f Ii:%f Io:%f Ps:%f\r\n",
    //                Vi, Vo, Pi, Ii, Io, Ps);

    //         uart_index = 0;  // Reset for next message
    //     } else if (uart_index < SUPERCAP_RX_BUFFER_SIZE - 1) {
    //         uart_buffer[uart_index++] = uart_byte;
    //     } else {
    //         uart_index = 0;  // Overflow protection
    //     }
    // }
}

// void parse_uart_data(const char *msg) {
//     // Example format:
//     // Vi:12.34 Vo:12.34 Pi:56.78 Io:2.34 Ps:50.00


//     sscanf(msg, "Vi:%f Vo:%f Pi:%f Io:%f Ps:%f", &Vi, &Vo, &Pi, &Io, &Ps);


//     // Now you can use the parsed variables as needed
// }

// void uart_receive_loop() {
//     while (1) {
//         HAL_UART_Receive(&huart1, &uart_byte, 1, HAL_MAX_DELAY);


//         if (uart_byte == '\n') {
//             uart_buffer[uart_index] = '\0';  // Null-terminate
//             parse_uart_data(uart_buffer);
//             uart_index = 0;  // Reset for next line
//         }
//         else if (uart_index < UART_BUFFER_SIZE - 1) {
//             uart_buffer[uart_index++] = uart_byte;
//         } else {
//             uart_index = 0;  // Overflow protection
//         }
//     }
// }


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
//    supercap_uart_instance_ptr->rx_buffer_size = 40; // Start with 'P' for power command
//    switch(Referee_System.Robot_State.Chassis_Power_Max) {
//         case 45:
//             supercap_uart_instance_ptr->rx_buffer = (uint8_t *)"P045P";
//             break;
//         case 50:
//            supercap_uart_instance_ptr->rx_buffer = (uint8_t *)"P050P";
//             break;
//         case 55:
//            supercap_uart_instance_ptr->rx_buffer = (uint8_t *)"P055P";
//             break;
//         // case 60:
//         //     g_swerve_constants.max_speed = MAX_SPEED_W60;
//         //     g_spintop_omega = SPINTOP_OMEGA_W60;
//         //     break;
//         // case 65:
//         //     g_swerve_constants.max_speed = MAX_SPEED_W65;
//         //     g_spintop_omega = SPINTOP_OMEGA_W65;
//         //     break;
//         // case 70:
//         //     g_swerve_constants.max_speed = MAX_SPEED_W70;
//         //     g_spintop_omega = SPINTOP_OMEGA_W70;
//         //     break;
//         // case 75:
//         //     g_swerve_constants.max_speed = MAX_SPEED_W75;
//         //     g_spintop_omega = SPINTOP_OMEGA_W75;
//         //     break;
//         // case 80:
//         //     g_swerve_constants.max_speed = MAX_SPEED_W80;
//         //     g_spintop_omega = SPINTOP_OMEGA_W80;
//         //     break;
//         // case 90:
//         //     g_swerve_constants.max_speed = MAX_SPEED_W90;
//         //     g_spintop_omega = SPINTOP_OMEGA_W90;
//         //     break;
//         // case 100:
//         //     g_swerve_constants.max_speed = MAX_SPEED_W100;
//         //     g_spintop_omega = SPINTOP_OMEGA_W100;
//         //     break;
//         // default:
//             g_swerve_constants.max_speed = MAX_SPEED_W45;
//             g_spintop_omega = SPINTOP_OMEGA_W45;
//     }
//     // CAN_Transmit(supercap_can_instance);
//     UART_Transmit(supercap_uart_instance_ptr, g_supercap.tx_buffer,sizeof(g_supercap.tx_buffer), UART_DMA);
}