// Referenced https://github.com/PurdueRM/wlb/blob/main/src/app/src/board_comm_task.c

#include "c_board_comm.h"
#include "bsp_can.h"
#include "robot.h"
#include "referee_system.h"
#include "supercap.h"
Board_Comm_Package_t g_board_comm_package; // 
CAN_Instance_t *g_board_communication; // the master board 
// CAN_Instance_t *g_board_slave; // the slave board
extern Referee_System_t Referee_System;
extern Supercap_t g_supercap;
uint8_t g_board_comm_package_first_part_established = 0; // flag to check if the first part of the package is established
uint8_t g_board_comm_package_second_part_established = 0; // flag to check if the second part of the package is established
uint8_t master_debug = 0;
uint8_t slave_debug = 0;
float entered_send = 0.0;

uint8_t g_board_comm_sending_pending = 0;

/**
 * @brief
 * @param
 */
void C_Board_Comm_Task_Init() {
    #ifdef MASTER 
        #pragma message "Master C_Board_Comm_Task_Init() is complied"
        // g_board_master = CAN_Device_Register(1, 0x310, 0x300, C_Board_Recv_Supercap_Info);
        // g_board_slave = CAN_Device_Register(1, 0x301, 0x311, C_Board_Recv_Ref_Info);

        master_debug = 1;

        g_board_communication = CAN_Device_Register(1, 0x310, 0x300, C_Board_Recv_Supercap_Info);
    #else 
        #pragma message "Slave C_Board_Comm_Task_Init() is compiled"
        // g_board_slave = CAN_Device_Register(1, 0x300, 0x310, C_Board_Recv_Supercap_Info);
        // g_board_master = CAN_Device_Register(1, 0x311, 0x301, C_Board_Recv_Ref_Info);

        slave_debug = 1;
        g_board_communication = CAN_Device_Register(1, 0x300, 0x310, C_Board_Recv_Ref_Info);
    #endif
}

/**
 * @brief
 * @param
 */
void C_Board_Recv_Ref_Info(CAN_Instance_t *can_instance)
{
    g_board_comm_package_first_part_established = 1; //debugging
    memcpy(&g_board_comm_package.power_limit, can_instance->rx_buffer, sizeof(float));
}

/**
 * @brief
 * @param
 */
void C_Board_Recv_Supercap_Info(CAN_Instance_t* can_instance)
{
    g_board_comm_package_second_part_established = 1; // debugging
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
        memcpy(&(g_board_communication->tx_buffer[0]), &(temp_power_limit), sizeof(float)); // &(Referee_System.Robot_State.Chassis_Power_Max)
        g_board_comm_sending_pending = 1;
    #else
        memcpy(&(g_board_communication->tx_buffer[0]), &(g_supercap.Vo), sizeof(float)); // Vo
        memcpy(&(g_board_communication->tx_buffer[sizeof(float)]), &(g_supercap.Ps), sizeof(float)); // Ps
        g_board_comm_sending_pending = 1;
    #endif

    entered_send += 0.0001; // TODO DELETE LATER AFTER DEBUGGING. CAUSES MEMORY ERROR
}
