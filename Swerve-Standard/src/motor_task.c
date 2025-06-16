#include "motor_task.h"
#include "dji_motor.h"
// #include "dm_motor.h"
// #include "mf_motor.h"
#include "supercap.h"
#include "c_board_comm.h"
extern Supercap_t g_supercap;
extern CAN_Instance_t* g_board_communication;
// extern CAN_Instance_t* g_board_master;

void Motor_Task_Loop() {
    #ifdef MASTER
        DJI_Motor_Send();
        CAN_Transmit(g_board_communication);
    #else 
        Supercap_Send();
        CAN_Transmit(g_board_communication);
    #endif
}

