/**
  ******************************************************************************
  * @file    remote.c
  * @brief   This file contains the driver functions for DR16 Remote From DJI
  ******************************************************************************
  */
#include "remote.h"
#include "bsp_uart.h"
#include "bsp_daemon.h"
#include <memory.h>

Remote_t g_remote;
UART_Instance_t *g_remote_uart;
Daemon_Instance_t *g_remote_daemon;

uint8_t remote_buffer[18];
volatile uint8_t debug_buffer[18];
/*
 * Remote_BufferProcess()
 * 
 * Decode the buffer received from DR16 receiver to g_remote. @ref Remote_t
 */
void Remote_Buffer_Process()
{
	memcpy((void*) debug_buffer, remote_buffer, 18);
	// controller decode
	g_remote.controller.right_stick.x = ((remote_buffer[0] | (remote_buffer[1] << 8)) & 0x07ff) - 1024;
	g_remote.controller.right_stick.y = (((remote_buffer[1] >> 3) | (remote_buffer[2] << 5)) & 0x07ff) - 1024;
	g_remote.controller.left_stick.x = (((remote_buffer[2] >> 6) | (remote_buffer[3] << 2) | (remote_buffer[4] << 10)) & 0x07ff) - 1024;
	g_remote.controller.left_stick.y = (((remote_buffer[4] >> 1) | (remote_buffer[5] << 7)) & 0x07ff) - 1024;
	g_remote.controller.wheel = ((remote_buffer[16] | (remote_buffer[17] << 8)) & 0x07FF) - 1024;
	g_remote.controller.left_switch = ((remote_buffer[5] >> 4) & 0x000C) >> 2;
	g_remote.controller.right_switch = ((remote_buffer[5] >> 4) & 0x0003);

	// mouse decode
	g_remote.mouse.x = (remote_buffer[6]) | (remote_buffer[7] << 8);
	g_remote.mouse.y = remote_buffer[8] | (remote_buffer[9] << 8);
	g_remote.mouse.z = remote_buffer[10] | (remote_buffer[11] << 8);
	g_remote.mouse.left = remote_buffer[12];
	g_remote.mouse.right = remote_buffer[13];

	// key decode
	uint16_t key_buffer = remote_buffer[14] | (remote_buffer[15] << 8);
	g_remote.keyboard.W = (key_buffer >> 0) & 0x001;
	g_remote.keyboard.S = (key_buffer >> 1) & 0x001;
	g_remote.keyboard.A = (key_buffer >> 2) & 0x001;
	g_remote.keyboard.D = (key_buffer >> 3) & 0x001;
	g_remote.keyboard.Shift = (key_buffer >> 4) & 0x001;
	g_remote.keyboard.Ctrl = (key_buffer >> 5) & 0x001;
	g_remote.keyboard.Q = (key_buffer >> 6) & 0x001;
	g_remote.keyboard.E = (key_buffer >> 7) & 0x001;
	g_remote.keyboard.R = (key_buffer >> 8) & 0x001;
	g_remote.keyboard.F = (key_buffer >> 9) & 0x001;
	g_remote.keyboard.G = (key_buffer >> 10) & 0x001;
	g_remote.keyboard.Z = (key_buffer >> 11) & 0x001;
	g_remote.keyboard.X = (key_buffer >> 12) & 0x001;
	g_remote.keyboard.C = (key_buffer >> 13) & 0x001;
	g_remote.keyboard.V = (key_buffer >> 14) & 0x001;
	g_remote.keyboard.B = (key_buffer >> 15) & 0x001;

	g_remote.online_flag = 1;
}

void Remote_Rx_Callback(UART_Instance_t *uart_instance)
{
	Remote_Buffer_Process();
	Daemon_Reload(g_remote_daemon);
}

void Remote_Timeout_Callback()
{
	// reinitalize the remote uart transmission
	UART_Service_Init(g_remote_uart);
	g_remote.online_flag = 0;
}

Remote_t* Remote_Init(UART_HandleTypeDef *huart)
{
	g_remote_uart = UART_Register(huart, remote_buffer, 18, Remote_Rx_Callback);
	g_remote_daemon = Daemon_Register(20, 20, Remote_Timeout_Callback);
	return &g_remote;
}