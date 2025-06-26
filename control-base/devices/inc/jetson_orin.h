#ifndef JETSON_ORIN_H
#define JETSON_ORIN_H

#include "bsp_uart.h"
#include "referee_system.h"

#define ORIN_DATA_RX_BUFER_SIZE (20)
#define ORIN_DATA_TX_BUFER_SIZE (40)

#define ORIN_TIMEOUT_MS (500)
#define ORIN_NAV_TIMEOUT_MS (200)
#define JETSON_ORIN_PERIOD (4)
#pragma message "orin period must match with algorithm team"

typedef struct
{

    uint8_t rx_buffer[ORIN_DATA_RX_BUFER_SIZE];
    uint8_t tx_buffer[ORIN_DATA_TX_BUFER_SIZE];

    struct
    {
        uint8_t header; // header byte 0xAA
        uint8_t enemy_color_is_red; // 1 for red and 0 for blue
        uint8_t game_status; // 0 for not started, 1 for preperation stage, 2 for 15 seconds referee check, 3 for 5 seconds count down, 4 for match going, 5 for calculating match result
        uint8_t rfid; // bit 0 for resupply, bit 1 for center zone
        float pitch_angle;        // rad
        float pitch_angular_rate; // rad/s
        float yaw_angular_rate;   // rad/s
        float position_x;         // m
        float position_y;         // m
        float orientation;        // rad
        float velocity_x;         // m/s
        float velocity_y;         // m/s
        int16_t HP;
        uint16_t reserved; // reserved for future use
    } sending;

    struct
    {
        uint8_t frame_id;
        uint8_t frame_type; // 0 for autoaiming, 1 for navigation, 2 for heart beat

        struct
        {
            float yaw;
            float pitch;
            uint8_t fire;
        } auto_aiming;

        struct
        {
            float x_vel;            // m/s
            float y_vel;            // m/s
            float yaw_angular_rate; // rad/s
            uint8_t state;          // 0 for stationary, 1 for moving, 2 for spinning
        } navigation;

        struct
        {
            uint8_t a;
            uint8_t b;
            uint8_t c;
            uint8_t d;
        } heart_beat;

        union
        {
            float data[3];
            uint8_t data_bytes[12];
        } float_byte;
    } receiving;

    uint8_t new_data_flag; // 1 for new data received, 0 for no new data
    uint8_t online_flag;
    uint8_t nav_online_flag; // 1 for navigation data received, 0 for no navigation data
} Jetson_Orin_Data_t;

extern Jetson_Orin_Data_t g_orin_data;
void Jetson_Orin_Init(UART_HandleTypeDef *huartx);
void Jetson_Orin_Send_Data(void);

#endif
