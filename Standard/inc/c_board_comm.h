// Referenced https://github.com/PurdueRM/wlb/blob/main/src/app/inc/board_comm_task.h
#ifndef __C_BOARD_COMM_H
#define __C_BOARD_COMM_H

// Define/undefine this as needed when flashing 
// the two c boards on the swerve robot
#define MASTER
#define C_BOARD_COMM_PERIOD (100) // ms

#include <stdint.h>
#include "bsp_uart.h"
#include "bsp_can.h"

typedef struct Board_Comm_Package_s 
{
    // from slave to master
    float Vo; // the voltage of the super cap
    float Ps; // power reference from the super cap

    // from master to slave
    uint16_t power_limit; //send power from the master to the slave
    uint8_t chassis_powered_on; // send chassis power state from master to slave
} Board_Comm_Package_t;

void C_Board_Comm_Task_Init();
void C_Board_Recv_Ref_Info(CAN_Instance_t *can_instance);
void C_Board_Recv_Supercap_Info(CAN_Instance_t *can_instance); // send 
void C_Board_Comm_Send_Loop();

#endif