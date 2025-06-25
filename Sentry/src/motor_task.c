#include "motor_task.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "mf_motor.h"
#include "supercap.h"
#include "c_board_comm.h"
extern uint8_t g_board_comm_sending_pending;
extern Supercap_t g_supercap;
uint8_t result_can = 0;
extern CAN_Instance_t* g_board_communication;
void Motor_Task_Loop() {
    DJI_Motor_Send();
    // MF_Motor_Send();
    DM_Motor_Send();

    g_supercap.send_counter++;
    if (g_supercap.send_counter >= 100) {
        Supercap_Send();
        g_supercap.send_counter = 0;
    }
    if (g_board_comm_sending_pending == 1)
    {
        result_can = CAN_Transmit(g_board_communication);
        g_board_comm_sending_pending = 0;
    }
}

