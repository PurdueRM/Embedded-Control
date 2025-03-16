/**
 ******************************************************************************
 * @file    remote.h
 * @brief   This is the header file for Remote.h
 ******************************************************************************
 */
#ifndef __REMOTE_H
#define __REMOTE_H

#include <stdint.h>
#include "bsp_uart.h"

#define REMOTE_STICK_MAX (660.0f)
#define REMOTE_ONLINE (1)
#define REMOTE_OFFLINE (0)

#define KEY_W ((uint16_t)0x01<<0)
#define KEY_S ((uint16_t)0x01<<1)
#define KEY_A ((uint16_t)0x01<<2)
#define KEY_D ((uint16_t)0x01<<3)
#define KEY_SHIFT ((uint16_t)0x01<<4)
#define KEY_CTRL ((uint16_t)0x01<<5)   
#define KEY_Q ((uint16_t)0x01<<6)      
#define KEY_E ((uint16_t)0x01<<7)      
#define KEY_R ((uint16_t)0x01<<8)    
#define KEY_F ((uint16_t)0x01<<9)
#define KEY_G ((uint16_t)0x01<<10)     
#define KEY_Z ((uint16_t)0x01<<11)    
#define KEY_X ((uint16_t)0x01<<12)
#define KEY_C ((uint16_t)0x01<<13)
#define KEY_V ((uint16_t)0x01<<14)
#define KEY_B ((uint16_t)0x01<<15)  

enum SwitchPos {UP=1, DOWN, MID};

struct Joystick
{
    int16_t x;
    int16_t y;
};

struct Controller
{
    struct Joystick left_stick;
    struct Joystick right_stick;
    int32_t wheel;
    uint8_t left_switch; // 1 up; 2 down; 3 mid
    uint8_t right_switch;
};

struct Keyboard
{
    uint8_t W;
    uint8_t S;
    uint8_t A;
    uint8_t D;
    uint8_t Q;
    uint8_t E;
    uint8_t Shift;
    uint8_t Ctrl;
    uint8_t R;
    uint8_t F;
    uint8_t G;
    uint8_t Z;
    uint8_t X;
    uint8_t C;
    uint8_t V;
    uint8_t B;
};

struct Mouse
{
    int16_t x;
    int16_t y;
    int16_t z;
    uint8_t left;
    uint8_t right;
};

typedef struct
{
    struct Controller controller;
    struct Keyboard keyboard;
    struct Mouse mouse;

    uint8_t online_flag;
} Remote_t;


extern Remote_t g_remote;
Remote_t* Remote_Init(UART_HandleTypeDef *huart);

#endif /* __REMOTE_H */
