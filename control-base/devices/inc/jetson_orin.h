#ifndef JETSON_ORIN_H
#define JETSON_ORIN_H

#include "bsp_uart.h"
#include "referee_system.h"

#define ORIN_DATA_RX_BUFER_SIZE (20)
#define ORIN_DATA_TX_BUFER_SIZE (34)

#define ORIN_TIMEOUT_MS (3000)
#define JETSON_ORIN_PERIOD (100)

typedef struct
{

    uint8_t rx_buffer[ORIN_DATA_RX_BUFER_SIZE];
    uint8_t tx_buffer[ORIN_DATA_TX_BUFER_SIZE];

    struct
    {
        float pitch_angle;        // rad
        float pitch_angular_rate; // rad/s
        float yaw_angular_rate;   // rad/s
        float position_x;         // m
        float position_y;         // m
        float orientation;        // rad
        float velocity_x;         // m/s
        float velocity_y;         // m/s
        uint8_t game_start_flag; //1 for start and 0 for not start
		uint8_t enemy_color_flag; //1 for red and 0 for blue

        union
        {
            float data[8];
            uint8_t data_bytes[32];
        } float_byte;
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

} Jetson_Orin_Data_t;

extern Jetson_Orin_Data_t g_orin_data;
void Jetson_Orin_Init(UART_HandleTypeDef *huartx);
void Jetson_Orin_Send_Data(void);

#endif
