// #include "custom_controller.h"
// #include "referee_system.h"

// #include <stdint.h>
// #include <string.h>

// custom_controller_t g_custom_controller;

// static float unpack_3byte_float(const uint8_t *src) {
//     uint32_t val = 0;
    
//     val |= ((uint32_t)src[0] << 8);
//     val |= ((uint32_t)src[1] << 16);
//     val |= ((uint32_t)src[2] << 24);

//     float f;
//     memcpy(&f, &val, sizeof(float)); 
//     return f;
// }

// void parse_controller_data(const uint8_t *packet_buffer) {

//     int idx = 7;
    
//     for (int i = 0; i < 3; i++) { 
//         g_custom_controller->angle[i] = unpack_3byte_float(&packet_buffer[idx]); 
//         idx += 3; 
//     }
//     for (int i = 0; i < 3; i++) { 
//         g_custom_controller->acc[i] = unpack_3byte_float(&packet_buffer[idx]); 
//         idx += 3; 
//     }
//     for (int i = 0; i < 3; i++) { 
//         g_custom_controller->ang_vel[i] = unpack_3byte_float(&packet_buffer[idx]); 
//         idx += 3; 
//     }

//     // 4. Extract Joystick
//     g_custom_controller->joystick_x = packet_buffer[idx++];
//     g_custom_controller->joystick_y = packet_buffer[idx++];
    
//     // 5. Extract & Decode Flags
//     uint8_t raw_flags = packet_buffer[idx++];
    
//     g_custom_controller->flags.reset_heading = (raw_flags >> 0) & 0x01;
//     g_custom_controller->flags.teleop_toggle = (raw_flags >> 1) & 0x01;
//     g_custom_controller->flags.ee_trigger    = (raw_flags >> 2) & 0x01;
//     g_custom_controller->flags.ee_twist      = (raw_flags >> 3) & 0x01;
// }