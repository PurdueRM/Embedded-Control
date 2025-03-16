#include "mf_motor.h"
#include "bsp_can.h"
#include "user_math.h"
#include <stdlib.h>

#pragma message "Check Max Device Number"
#define MF_MAX_DEVICE (2)
MF_Motor_Handle_t *g_mf_motors[MF_MAX_DEVICE] = {NULL};
uint8_t g_mf_motor_num = 0;

void MF_Motor_Decode(CAN_Instance_t *can_instance);

MF_Motor_Handle_t *MF_Motor_Init(MF_Motor_Config_t config)
{
    MF_Motor_Handle_t *motor = malloc(sizeof(MF_Motor_Handle_t));
    motor->can_bus = config.can_bus;
    motor->control_mode = config.control_mode;
    motor->tx_id = config.tx_id;
    motor->rx_id = config.rx_id;
    motor->stats = malloc(sizeof(MF_Motor_Stats_t));
    motor->can_instance = CAN_Device_Register(config.can_bus, config.tx_id, config.rx_id, MF_Motor_Decode);
    motor->can_instance->binding_motor_stats = (void *)motor->stats;
    motor->target_pos = 0;
    motor->target_vel = 0;
    motor->target_torq = 0;
    motor->stats->enabled = 0;
    motor->stats->angle = 0;
    motor->stats->velocity = 0;
    motor->stats->current = 0;
    motor->stats->temp = 0;
    motor->stats->kp_ang = 0;
    motor->stats->ki_ang = 0;
    motor->stats->kp_vel = 0;
    motor->stats->ki_vel = 0;
    motor->stats->kp_torq = 0;
    motor->stats->ki_torq = 0;

    g_mf_motors[g_mf_motor_num++] = motor;
    return motor;
}

void MF_Motor_Decode(CAN_Instance_t *can_instance)
{
    uint8_t *data = can_instance->rx_buffer;
    MF_Motor_Stats_t *motor_info = (MF_Motor_Stats_t *)can_instance->binding_motor_stats;

    switch (data[0])
    {
    case 0x30:
        motor_info->kp_ang = data[2];
        motor_info->ki_ang = data[3];
        motor_info->kp_vel = data[4];
        motor_info->ki_vel = data[5];
        motor_info->kp_torq = data[6];
        motor_info->ki_torq = data[7];
        break;
    case 0x80:
        motor_info->enabled = 0;
        break;

    case 0x88:
        motor_info->enabled = 1;
        break;

    case 0xA1:
    case 0xA2:
        motor_info->angle = (data[7] << 8) + data[6];
        motor_info->velocity = (data[5] << 8) + data[4];
        motor_info->current = (data[3] << 8) + data[2];
        motor_info->temp = data[1];
        break;
    }
}

void MF_Motor_EnableMotor(MF_Motor_Handle_t *motor)
{
    // store local pointer to avoid multiple dereference
    uint8_t *tx_buffer = motor->can_instance->tx_buffer;
    tx_buffer[0] = 0x88;
    memset(&tx_buffer[1], 0, 7);

    // set the flag to send the data
    motor->send_pending_flag = 1;
}

void MF_Motor_DisableMotor(MF_Motor_Handle_t *motor)
{
    // store local pointer to avoid multiple dereference
    uint8_t *tx_buffer = motor->can_instance->tx_buffer;
    tx_buffer[0] = 0x80;
    memset(&tx_buffer[1], 0, 7);

    // set the flag to send the data
    motor->send_pending_flag = 1;
}

void MF_Motor_GetPIDParam(MF_Motor_Handle_t *motor)
{
    // store local pointer to avoid multiple dereference
    uint8_t *tx_buffer = motor->can_instance->tx_buffer;
    tx_buffer[0] = 0x30;
    memset(&tx_buffer[1], 0, 7);

    // set the flag to send the data
    motor->send_pending_flag = 1;
}

void MF_Motor_PIDToRam(MF_Motor_Handle_t *motor,
                       uint8_t kp_ang, uint8_t ki_ang,
                       uint8_t kp_vel, uint8_t ki_vel,
                       uint8_t kp_torq, uint8_t ki_torq)
{
    // store local pointer to avoid multiple dereference
    uint8_t *tx_buffer = motor->can_instance->tx_buffer;
    tx_buffer[0] = 0x31;
    tx_buffer[1] = 0x00;
    tx_buffer[2] = kp_ang;
    tx_buffer[3] = ki_ang;
    tx_buffer[4] = kp_vel;
    tx_buffer[5] = ki_vel;
    tx_buffer[6] = kp_torq;
    tx_buffer[7] = ki_torq;

    // set the flag to send the data
    motor->send_pending_flag = 1;
}

/**
 * @brief  Closed loop torque control for MF motor.
 * @param  motor: Pointer to the MF motor handle structure.
 * @param  torq: torque value
 *                  - range from -2000 to 2000 (motor can reply +-2048)
 *                      MF Motors: 2048 represents 16.5A
 *                      MG Motors: 2048 represents 33A
 *                  - Check motor datasheet for torque constant
 */
void MF_Motor_TorqueCtrl(MF_Motor_Handle_t *motor, int16_t torq)
{
    __MAX_LIMIT(torq, -2048, 2048);
    // store local pointer to avoid multiple dereference
    uint8_t *tx_buffer = motor->can_instance->tx_buffer;
    // tx_buffer[0] = 0xA1;
    // tx_buffer[1] = 0x00;
    // tx_buffer[2] = 0x00;
    // tx_buffer[3] = 0x00;
    // tx_buffer[4] = torq & 0xFF;
    // tx_buffer[5] = (torq >> 8) & 0xFF;
    // tx_buffer[6] = 0x00;
    // tx_buffer[7] = 0x00;
    memset(tx_buffer, 0, 8); // more efficient than setting each byte to 0
    tx_buffer[0] = 0xA1;
    tx_buffer[4] = torq & 0xFF;
    tx_buffer[5] = (torq >> 8) & 0xFF;

    // set the flag to send the data
    motor->send_pending_flag = 1;
}

/**
 * @brief Closed loop velocity control for MF motor.
 * @param motor Pointer to the MF motor handle structure.
 * @param vel Desired velocity setpoint for the motor.
 *             - unit: 0.01dps/LSB
 *
 * @note The function packages the velocity command into a CAN bus frame.
 *       DATA[0] is set to command identifier 0xA2.
 *       DATA[4] to DATA[7] are assigned the velocity value, split into 4 bytes.
 *       This function uses a local buffer to avoid multiple dereferences of the motor handle.
 *
 * @warning motor respond velocity in 1dps/LSB
 */
void MF_Motor_VelocityCtrl(MF_Motor_Handle_t *motor, int32_t vel)
{
    // store local pointer to avoid multiple dereference
    uint8_t *tx_buffer = motor->can_instance->tx_buffer;
    // tx_buffer[0] = 0xA2;
    // tx_buffer[1] = 0x00;
    // tx_buffer[2] = 0x00;
    // tx_buffer[3] = 0x00;
    // tx_buffer[4] = vel & 0xFF;
    // tx_buffer[5] = (vel >> 8) & 0xFF;
    // tx_buffer[6] = (vel >> 16) & 0xFF;
    // tx_buffer[7] = (vel >> 24) & 0xFF;
    memset(tx_buffer, 0, 8); // more efficient than setting each byte to 0
    tx_buffer[0] = 0xA2;
    tx_buffer[4] = vel & 0xFF;
    tx_buffer[5] = (vel >> 8) & 0xFF;
    tx_buffer[6] = (vel >> 16) & 0xFF;
    tx_buffer[7] = (vel >> 24) & 0xFF;

    // set the flag to send the data
    motor->send_pending_flag = 1;
}

/**
 * @brief Closed loop position control for MF motor.
 * @param motor Pointer to the MF motor handle structure.
 * @param pos Desired position setpoint for the motor.
 *             - Unit: 0.01 degree/LSB (36000 represents 360 degrees)
 *
 * @note The function packages the position command into a CAN bus frame.
 *       DATA[0] is set to the command identifier 0xA3.
 *       DATA[4] to DATA[7] are assigned the position value, split into 4 bytes.
 *       This function uses a local buffer to avoid multiple dereferences of the motor handle and initializes
 *       the buffer to zero before setting the command identifier and position value.
 */
void MF_Motor_PositionCtrl(MF_Motor_Handle_t *motor, int32_t pos)
{
    // store local pointer to avoid multiple dereference
    uint8_t *tx_buffer = motor->can_instance->tx_buffer;

    // tx_buffer[0] = 0xA3;
    // tx_buffer[1] = 0x00;
    // tx_buffer[2] = 0x00;
    // tx_buffer[3] = 0x00;
    // tx_buffer[4] = pos & 0xFF;
    // tx_buffer[5] = (pos >> 8) & 0xFF;
    // tx_buffer[6] = (pos >> 16) & 0xFF;
    // tx_buffer[7] = (pos >> 24) & 0xFF;

    memset(tx_buffer, 0, 8); // more efficient than setting each byte to 0
    tx_buffer[0] = 0xA3;
    tx_buffer[4] = pos & 0xFF;
    tx_buffer[5] = (pos >> 8) & 0xFF;
    tx_buffer[6] = (pos >> 16) & 0xFF;
    tx_buffer[7] = (pos >> 24) & 0xFF;

    // set the flag to send the data
    motor->send_pending_flag = 1;
}

void MF_Motor_Send(void)
{
    for (int i = 0; i < g_mf_motor_num; i++) // loop through all the motors
    {
        if (g_mf_motors[i]->send_pending_flag)
        {                                               // check if the flag is set
            CAN_Transmit(g_mf_motors[i]->can_instance); // send the data
            g_mf_motors[i]->send_pending_flag = 0;      // clear the flag
        }
    }
}
