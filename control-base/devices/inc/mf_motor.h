#ifndef MF_MOTOR_H
#define MF_MOTOR_H

#include "bsp_can.h"

#define SINGLE_MOTOR_CTRL_STD 0x140

typedef struct MF_MOTOR_INFO
{
    uint8_t enabled;
    uint16_t angle;
    int16_t velocity;
    int16_t current;
    int8_t temp;

    uint16_t kp_ang;
    uint16_t ki_ang;
    uint16_t kp_vel;
    uint16_t ki_vel;
    uint16_t kp_torq;
    uint16_t ki_torq;
} MF_Motor_Stats_t;

typedef struct _MF_Motor_Config
{
    uint8_t can_bus;
    uint8_t control_mode;
    uint32_t rx_id;
    uint32_t tx_id;

    float pos_offset;
    float kp_ang;
    float ki_ang;
    float kp_vel;
    float ki_vel;
    float kp_torq;
    float ki_torq;
} MF_Motor_Config_t;

typedef struct _MF_Motor
{
    /* CAN Information */
    uint8_t can_bus;
    uint8_t control_mode;
    uint8_t send_pending_flag;
    uint16_t tx_id;
    uint16_t rx_id;
    CAN_Instance_t *can_instance;

    /* Motor Target */
    float target_pos;
    float target_vel;
    float target_torq;

    /* Motor Stats */
    MF_Motor_Stats_t *stats;
} MF_Motor_Handle_t;

// TODO: document functions
void MF_Motor_Decode(CAN_Instance_t *can_instance);

// TODO: document functions
MF_Motor_Handle_t *MF_Motor_Init(MF_Motor_Config_t config);

// TODO: document functions
void MF_Motor_GetPIDParam(MF_Motor_Handle_t *motor);

// TODO: document functions
void MF_Motor_PIDToRam(MF_Motor_Handle_t *motor,
                       uint8_t kp_ang, uint8_t ki_ang,
                       uint8_t kp_vel, uint8_t ki_vel,
                       uint8_t kp_torq, uint8_t ki_torq);

// TODO: document functions
void MF_Motor_EnableMotor(MF_Motor_Handle_t *motor);

// TODO: document functions
void MF_Motor_DisableMotor(MF_Motor_Handle_t *motor);

// TODO: document functions
void MF_Motor_PIDToRam(MF_Motor_Handle_t *motor,
                       uint8_t kp_ang, uint8_t ki_ang,
                       uint8_t kp_vel, uint8_t ki_vel,
                       uint8_t kp_torq, uint8_t ki_torq);

/**
 * @brief  Closed loop torque control for MF motor.
 * @param  motor: Pointer to the MF motor handle structure.
 * @param  torq: torque value
 *                  - range from -2000 to 2000 (motor can reply +-2048)
 *                      MF Motors: 2048 represents 16.5A
 *                      MG Motors: 2048 represents 33A
 *                  - Check motor datasheet for torque constant
*/
void MF_Motor_TorqueCtrl(MF_Motor_Handle_t *motor, int16_t torq);

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
void MF_Motor_VelocityCtrl(MF_Motor_Handle_t *motor, int32_t vel);

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
void MF_Motor_PositionCtrl(MF_Motor_Handle_t *motor, int32_t pos);

/**
 * @brief Global function to send the motor control data.
*/
void MF_Motor_Send(void);
#endif
