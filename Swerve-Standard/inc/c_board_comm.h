// Referenced https://github.com/PurdueRM/wlb/blob/main/src/app/inc/board_comm_task.h
#ifndef __C_BOARD_COMM_H
#define __C_BOARD_COMM_H

// Define/undefine this as needed when flashing 
// the two c boards on the swerve robot
// #define MASTER
#define C_BOARD_COMM_PERIOD (10) // ms

#include <stdint.h>
#include "bsp_uart.h"

typedef struct Board_Comm_Package_s 
{
    // from slave to master
    float Vo; // the voltage of the super cap
    float Ps; // power reference from the super cap

    // from master to slave
    float power_limit; //send power from the master to the slave
} Board_Comm_Package_t;

void C_Board_Comm_Task_Init();
void C_Board_Recv_Ref_Info();
void C_Board_Recv_Supercap_Info(); // send 
void C_Board_Comm_Send_Loop();

#endif