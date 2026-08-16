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

Remote_t* Remote_Init(UART_HandleTypeDef *huart)
{
	g_remote_uart = &uart5_instance;
    g_remote.online_flag = REMOTE_OFFLINE;
	return &g_remote;
}

/*
 * Remote_BufferProcess()
 * 
 * Decode the buffer received from DR16 receiver to g_remote. @ref Remote_t
 */
void Remote_Buffer_Process()
{
    UART_Message_t received_msg;

    const TickType_t failsafe_timeout = pdMS_TO_TICKS(20); // 20 ms

    if (xQueueReceive(uart5_instance.msg_queue, &received_msg, failsafe_timeout) == pdTRUE) {
        if (received_msg.length != 18) {
            return; 
        }
	    memcpy(remote_buffer, (void*) received_msg.payload, 18);
        SBUS_TO_RC(remote_buffer, &g_remote);

        g_remote.online_flag = REMOTE_ONLINE;
    }
    else { // Receive timed out --> controller is disconnected
        memset(&g_remote, 0, sizeof(Remote_t));
        g_remote.online_flag = REMOTE_OFFLINE;
    }
}

/**
  * @brief  convert the remote control received message (Pointer Version)
  * @param  sbus_buf: pointer to a array that contains the information of the received message.
  * @param  remote_ctrl: pointer to a Remote_Info_Typedef structure that
  *         contains the information  for the remote control.
  * @retval none
  */
void SBUS_TO_RC(volatile const uint8_t *sbus_buf, Remote_t *remote_ctrl)
{
    if (sbus_buf == NULL || remote_ctrl == NULL) return;
    /* Channel 0, 1, 2, 3 */
    remote_ctrl->controller.right_stick.x = (  sbus_buf[0]       | (sbus_buf[1] << 8 ) ) & 0x07ff;                            //!< Channel 0
    remote_ctrl->controller.right_stick.y = ( (sbus_buf[1] >> 3) | (sbus_buf[2] << 5 ) ) & 0x07ff;                            //!< Channel 1
    remote_ctrl->controller.left_stick.x = ( (sbus_buf[2] >> 6) | (sbus_buf[3] << 2 ) | (sbus_buf[4] << 10) ) & 0x07ff;      //!< Channel 2
    remote_ctrl->controller.left_stick.y = ( (sbus_buf[4] >> 1) | (sbus_buf[5] << 7 ) ) & 0x07ff;                            //!< Channel 3
    remote_ctrl->controller.wheel = (  sbus_buf[16] 	   | (sbus_buf[17] << 8) ) & 0x07ff;                 			      //!< Channel 4
    /* Switch left, right */
    remote_ctrl->controller.left_switch = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                  //!< Switch left
    remote_ctrl->controller.right_switch = ((sbus_buf[5] >> 4) & 0x0003);             //!< Switch right
    /* Mouse axis: X, Y, Z */
    remote_ctrl->mouse.x = sbus_buf[6]  | (sbus_buf[7] << 8);                    //!< Mouse X axis
    remote_ctrl->mouse.y = sbus_buf[8]  | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    remote_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    /* Mouse Left, Right Is Press  */
    remote_ctrl->mouse.left = sbus_buf[12];                                  //!< Mouse Left Is Press
    remote_ctrl->mouse.right = sbus_buf[13];                                  //!< Mouse Right Is Press
    /* KeyBoard decode */
    uint16_t key_buffer = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard buffer
	remote_ctrl->keyboard.W     = (key_buffer >> 0)  & 0x0001;  //!< Key W
	remote_ctrl->keyboard.S     = (key_buffer >> 1)  & 0x0001;  //!< Key S
	remote_ctrl->keyboard.A     = (key_buffer >> 2)  & 0x0001;  //!< Key A
	remote_ctrl->keyboard.D     = (key_buffer >> 3)  & 0x0001;  //!< Key D
	remote_ctrl->keyboard.Shift = (key_buffer >> 4)  & 0x0001;  //!< Key Shift
	remote_ctrl->keyboard.Ctrl  = (key_buffer >> 5)  & 0x0001;  //!< Key Ctrl
	remote_ctrl->keyboard.Q     = (key_buffer >> 6)  & 0x0001;  //!< Key Q
	remote_ctrl->keyboard.E     = (key_buffer >> 7)  & 0x0001;  //!< Key E
	remote_ctrl->keyboard.R     = (key_buffer >> 8)  & 0x0001;  //!< Key R
	remote_ctrl->keyboard.F     = (key_buffer >> 9)  & 0x0001;  //!< Key F
	remote_ctrl->keyboard.G     = (key_buffer >> 10) & 0x0001;  //!< Key G
	remote_ctrl->keyboard.Z     = (key_buffer >> 11) & 0x0001;  //!< Key Z
	remote_ctrl->keyboard.X     = (key_buffer >> 12) & 0x0001;  //!< Key X
	remote_ctrl->keyboard.C     = (key_buffer >> 13) & 0x0001;  //!< Key C
	remote_ctrl->keyboard.V     = (key_buffer >> 14) & 0x0001;  //!< Key V
	remote_ctrl->keyboard.B     = (key_buffer >> 15) & 0x0001;  //!< Key B
	remote_ctrl->controller.right_stick.x -= RC_CH_VALUE_OFFSET;
    remote_ctrl->controller.right_stick.y -= RC_CH_VALUE_OFFSET;
    remote_ctrl->controller.left_stick.x -= RC_CH_VALUE_OFFSET;
    remote_ctrl->controller.left_stick.y -= RC_CH_VALUE_OFFSET;
    remote_ctrl->controller.wheel -= RC_CH_VALUE_OFFSET;
    
	/* reset the lost flag */
	remote_ctrl->online_flag = REMOTE_ONLINE;
}
