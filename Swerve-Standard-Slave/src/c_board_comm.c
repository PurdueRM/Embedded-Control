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
 * @brief Registers the callback functions and inits the CAN_Instance_t
 *        depending on if its the master or the slave board
 * @param
 */
void C_Board_Comm_Task_Init() {
    #ifdef MASTER 
        #pragma message "Master C_Board_Comm_Task_Init() is complied"
        // g_board_master = CAN_Device_Register(1, 0x310, 0x300, C_Board_Recv_Supercap_Info);
        // g_board_slave = CAN_Device_Register(1, 0x301, 0x311, C_Board_Recv_Ref_Info);

        master_debug = 1;

        g_board_communication = CAN_Device_Register(1, 0x10, 0x00, C_Board_Recv_Supercap_Info);
    #else 
        #pragma message "Slave C_Board_Comm_Task_Init() is compiled"
        // g_board_slave = CAN_Device_Register(1, 0x300, 0x310, C_Board_Recv_Supercap_Info);
        // g_board_master = CAN_Device_Register(1, 0x311, 0x301, C_Board_Recv_Ref_Info);

        // slave_debug = 1;
        g_board_communication = CAN_Device_Register(1, 0x00, 0x10, C_Board_Recv_Ref_Info);
    #endif
}

/**
 * @brief Callback function for receiving power limit info from 
 *        the master board
 * @param
 */
void C_Board_Recv_Ref_Info(CAN_Instance_t *can_instance)
{
    g_board_comm_package_first_part_established = 1; //debugging
    memcpy(&g_board_comm_package.power_limit, can_instance->rx_buffer, sizeof(uint16_t));
    memcpy(&g_board_comm_package.chassis_powered_on, &can_instance->rx_buffer[sizeof(uint16_t)], sizeof(uint8_t));
}

/**
 * @brief Callback function for receiving supercap info from
 *        the slave board
 * @param
 */
void C_Board_Recv_Supercap_Info(CAN_Instance_t* can_instance)
{
    g_board_comm_package_second_part_established = 1; // debugging
    memcpy(&g_board_comm_package.Vo, &can_instance->rx_buffer[0], sizeof(float));
    memcpy(&g_board_comm_package.Ps, &can_instance->rx_buffer[sizeof(float)], sizeof(float));
}

/**
 * @brief Send the info over the can.
 *        Make sure you call CAN_Transmit(g_board_communication)
 *        in the motor_tasks.
 *        If nothing is sending, please make sure the CAN/Board is powered
 *        with 5V or more. 3.3V was not enough
 * @param
 */
void C_Board_Comm_Send_Loop()
{
    #ifdef MASTER
        memcpy(&(g_board_communication->tx_buffer[0]), &(Referee_System.Robot_State.Chassis_Power_Max), sizeof(uint16_t)); // &(Referee_System.Robot_State.Chassis_Power_Max)
        g_board_comm_sending_pending = 1;
    #else
        memcpy(&(g_board_communication->tx_buffer[0]), &(g_supercap.Vo), sizeof(float)); // Vo
        memcpy(&(g_board_communication->tx_buffer[sizeof(float)]), &(g_supercap.Ps), sizeof(float)); // Ps
        // g_board_communication->tx_buffer[0] = 1;
        // g_board_communication->tx_buffer[1] = 2;
        // g_board_communication->tx_buffer[2] = 3;
        // g_board_communication->tx_buffer[3] = 4;
        // g_board_communication->tx_buffer[4] = 5;
        // g_board_communication->tx_buffer[5] = 6;
        // g_board_communication->tx_buffer[6] = 7;
        // g_board_communication->tx_buffer[7] = 8; // TODO: REMOVE THIS LATER. FOR DEBUGGING PURPOSES
        g_board_comm_sending_pending = 1;
    #endif

    // entered_send += 0.0001; // TODO DELETE LATER AFTER DEBUGGING. CAUSES MEMORY ERROR
}
