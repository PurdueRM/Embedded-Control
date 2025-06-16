// Referenced https://github.com/PurdueRM/wlb/blob/main/src/app/src/board_comm_task.c

#include "c_board_comm.h"
#include "bsp_can.h"
#include "robot.h"
#include "referee_system.h"
#include "supercap.h"
Board_Comm_Package_t g_board_comm_package; // 
CAN_Instance_t *g_board_master; // the master board 
CAN_Instance_t *g_board_slave; // the slave board
extern Referee_System_t Referee_System;
extern Supercap_t g_supercap;

/**
 * @brief
 * @param
 */
void C_Board_Comm_Task_Init() {
    #ifdef MASTER 
        #pragma message "Master C_Board_Comm_Task_Init() is complied"
        // g_board_master = CAN_Device_Register(1, 0x310, 0x300, C_Board_Recv_Supercap_Info);
        // g_board_slave = CAN_Device_Register(1, 0x301, 0x311, C_Board_Recv_Ref_Info);
        g_board_master = CAN_Device_Register(1, 0x310, 0x300, C_Board_Recv_Supercap_Info);
    #else 
        #pragma message "Slave C_Board_Comm_Task_Init() is compiled"
        // g_board_master = CAN_Device_Register(1, 0x300, 0x310, C_Board_Recv_Supercap_Info);
        // g_board_slave = CAN_Device_Register(1, 0x311, 0x301, C_Board_Recv_Ref_Info);
        g_board_slave = CAN_Device_Register(1, 0x300, 0x310, C_Board_Recv_Ref_Info);
    #endif
}

/**
 * @brief
 * @param
 */
void C_Board_Recv_Ref_Info(CAN_Instance_t *can_instance)
{
    memcpy(&g_board_comm_package.power_limit, can_instance->rx_buffer, sizeof(float));
}

/**
 * @brief
 * @param
 */
void C_Board_Recv_Supercap_Info(CAN_Instance_t* can_instance)
{
    memcpy(&g_board_comm_package.Vo, &can_instance->rx_buffer[0], sizeof(float));
    memcpy(&g_board_comm_package.Ps, &can_instance->rx_buffer[sizeof(float)], sizeof(float));
}

/**
 * @brief
 * @param
 */
void C_Board_Comm_Send_Loop()
{
    #ifdef MASTER
        float temp_power_limit = 69.0;
        memcpy(&(g_board_master->tx_buffer[0]), &(temp_power_limit), sizeof(float)); // &(Referee_System.Robot_State.Chassis_Power_Max)
    #else
        memcpy(&(g_board_slave->tx_buffer[0]), &(g_supercap.Vo), sizeof(float)); // Vo
        memcpy(&(g_board_slave->tx_buffer[sizeof(float)]), &(g_supercap.Ps), sizeof(float)); // Ps
    #endif
}
