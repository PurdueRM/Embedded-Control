#ifndef __SUPERCAP_H
#define __SUPERCAP_H

#include <stdint.h>
#include "bsp_can.h"
#include "referee_system.h"
#include "jetson_orin.h"
#include "bsp_uart.h"
// #define SUPERCAP_POWER (20)
#define SUPERCAP_SEND_FREQ (1.0f) // Hz
//todo ask why it is 128
#define SUPERCAP_RX_BUFFER_SIZE (1) //number of bytes 
#define SUPERCAP_BUFFER_SIZE (64) // new buffer
#define SUPERCAP_TX_BUFFER_SIZE (3) //todo finish this
typedef struct
{
    uint8_t can_bus;
    uint16_t tx_id;
    uint16_t rx_id;
    uint16_t send_counter;

    uint8_t receive_counter;
    uint8_t final_receive_counter;

    uint8_t supercap_percent;
    uint8_t supercap_enabled_flag;
    
    float Vi; //input voltage
    float Vo; //output voltage
    float Pi; //input power
    float Ii; //input current
    float Ps; //reference power
    float Io; //output current

    //formate of rx
    //printf("Vi:%2.2f Vo:%2.2f Pi:%3.2f Ii:%2.2f Io:%2.2f Pa:%3.2f\r\n", VIPMR, VOFWR,POW_IN, IIPWR, IOPWR, pref);
    uint8_t rx_buffer[SUPERCAP_RX_BUFFER_SIZE];
    uint8_t tx_buffer[SUPERCAP_TX_BUFFER_SIZE]; // chnage this to other uint type if possible
    uint8_t buffer_for_construction[SUPERCAP_BUFFER_SIZE]; //storing each byte unitl \r\n
    uint8_t final_buffer_for_construction[SUPERCAP_BUFFER_SIZE];
    // struct {
    //     float input_voltage; // Vi
    //     float output_voltage; // Vo
    //     float input_power; //Pi
    //     float input_current; //Ii
    //     float output_currenIi; //Io
    //     float ref_power; //Pa
    // } supercap_data;

} Supercap_t;


void Supercap_Init(UART_HandleTypeDef *huartx); // in robot the input element is g_supercap
void Supercap_Decode(UART_Instance_t *supercap_uart_instance_ptr, float *Vi, float *Vo, float *Pi, float *Ii, float *Io, float *Ps);
void Supercap_Send(void);

extern Supercap_t g_supercap;

#endif // __SUPERCAP_H