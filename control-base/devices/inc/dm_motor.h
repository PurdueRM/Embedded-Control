#ifndef DM_MOTOR_H
#define DM_MOTOR_H

#include <stdint.h>
#include "bsp_can.h"

#define P_MIN -12.5f
#define P_MAX 12.5f
#define V_MIN -30.0f
#define V_MAX 30.0f
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f
#define T_MIN -10.0f
#define T_MAX 10.0f

#define DM_MOTOR_MIT (0)
#define DM_MOTOR_POS_VEL (1)
#define DM_MOTOR_VEL (2)

#define DM_MOTOR_ZERO_CURRENT (0)
#define DM_MOTOR_HARDWARE_DISABLE (1)

typedef struct
{
    uint8_t id;
    uint8_t state;
    uint16_t pos_int;
    uint16_t vel_int;
    uint16_t torq_int;

    float pos;
    float pos_raw;
    float pos_offset;
    float vel;
    float torq;
    uint16_t t_mos;
    uint16_t t_rotor;

} DM_Motor_Stats_t;

typedef struct _DM_Motor_Config {
    uint8_t can_bus;
    uint8_t control_mode;
    uint32_t rx_id;
    uint32_t tx_id;
    uint8_t disable_behavior; /* DM_MOTOR_ZERO_CURRENT or DM_MOTOR_HARDWARE_DISABLE
    DM_MOTOR_ZERO_CURRENT: set a zero current to the motor
    DM_MOTOR_HARDWARE_DISABLE: disable the motor by sending disable signal to motor driver
    
    */
    float pos_offset;
    float kp;
    float kd;
} DM_Motor_Config_t;

typedef struct _DM_Motor {
    /* CAN Information */
    uint8_t can_bus;
    uint8_t control_mode;
    uint8_t send_pending_flag;
    uint16_t tx_id;
    uint16_t rx_id;
    uint8_t disable_behavior;
    CAN_Instance_t *can_instance;
    
    /* Motor Target */
    float target_pos;
    float target_vel;
    float kp;
    float kd;
    float torq;

    /* Motor Sensor Feedback */
    DM_Motor_Stats_t *stats;
} DM_Motor_Handle_t;

void DM_Motor_Disable_Motor(DM_Motor_Handle_t *motor);
void DM_Motor_Enable_Motor(DM_Motor_Handle_t *motor);
DM_Motor_Handle_t* DM_Motor_Init(DM_Motor_Config_t *config);
void DM_Motor_Ctrl_MIT(DM_Motor_Handle_t *motor, float target_pos, float target_vel, float torq);
void DM_Motor_Set_MIT_PD(DM_Motor_Handle_t *motor, float kp, float kd);

/**
 * Global function to send the motor control data
*/
void DM_Motor_Send(void);
#endif
