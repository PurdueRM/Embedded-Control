#include "motor_task.h"
#include "dji_motor.h"
// #include "dm_motor.h"
// #include "mf_motor.h"
#include "supercap.h"
#include "c_board_comm.h"
#include "bsp_daemon.h"
#include "bsp_can.h"
extern Supercap_t g_supercap;
extern CAN_Instance_t* g_board_communication;
// extern CAN_Instance_t* g_board_master;
extern uint8_t g_board_comm_sending_pending;
uint8_t result_can = 0;
Daemon_Instance_t *g_motor_task_daemon_ptr = NULL;
#define MOTOR_TIMEOUT_MS (100) // 1 second timeout for motor task

void Motor_Task_Timeout_Callback(void) {
    CAN_Service_Restart();
}

void Motor_Task_Init() {
    uint16_t reload_value = MOTOR_TIMEOUT_MS / DAEMON_PERIOD;
	uint16_t initial_counter = reload_value;
	g_motor_task_daemon_ptr = Daemon_Register(reload_value, initial_counter, Motor_Task_Timeout_Callback);
}


void Motor_Task_Loop() {
    Daemon_Reload(g_motor_task_daemon_ptr);
    #ifdef MASTER
        DJI_Motor_Send();
    #else
        if (g_board_comm_sending_pending == 1)
        {
            Supercap_Send();
        }
    #endif
    if (g_board_comm_sending_pending == 1)
    {
        result_can = CAN_Transmit(g_board_communication);
        g_board_comm_sending_pending = 0;
    }
}

