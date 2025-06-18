#include "motor_task.h"
#include "dji_motor.h"
// #include "dm_motor.h"
// #include "mf_motor.h"
#include "supercap.h"
#include "c_board_comm.h"
extern Supercap_t g_supercap;
extern CAN_Instance_t* g_board_communication;
// extern CAN_Instance_t* g_board_master;
extern uint8_t g_board_comm_sending_pending;
uint8_t result_can = 0;
void Motor_Task_Loop() {

    if (g_board_comm_sending_pending == 1)
    {
        result_can = CAN_Transmit(g_board_communication);
        g_board_comm_sending_pending = 0;
    }
}

