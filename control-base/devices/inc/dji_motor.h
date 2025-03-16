#ifndef DJI_MOTOR_H
#define DJI_MOTOR_H

#include <stdint.h>
#include "bsp_can.h"
#include "motor.h"

#define MAX_DJI_MOTORS (16)      // realloac will be called to shrink the array
#define MAX_DJI_MOTOR_GROUPS (6) // realloac will be called to shrink the array
#define DJI_TX_ID_PLACEHOLDER (0x00)

#define GM6020_MAX_CURRENT (28000) // -30000 ~ 30000
#define M3508_MAX_CURRENT (16000)  // -16384 ~ 16384
#define M2006_MAX_CURRENT (9000)   // -10000 ~ 10000

#define DJI_MAX_TICKS (8191.0f)
#define DJI_HALF_MAX_TICKS (4096)

#define M3508_REDUCTION_RATIO (187.0f / 3591.0f)
#define GM6020_REDUCTION_RATIO (1)
#define M2006_REDUCTION_RATIO (1.0f / 36.0f)

typedef enum DJI_Motor_Type
{
    GM6020, // -30000 to 30000
    M3508,
    M2006,
} DJI_Motor_Type_t;

typedef struct DJI_Motor_Stats_s
{
    /* CAN Frame Info */
    uint16_t current_tick;
    uint16_t last_tick;
    float current_vel_rpm;
    int16_t current_torq;
    uint8_t temp;

    /* Function Varaibles */
    uint16_t encoder_offset;
    int32_t total_round;
    float absolute_angle_rad;
    float last_absolute_angle_rad;
    float total_angle_rad;
    float reduction_ratio;
} DJI_Motor_Stats_t;

typedef struct dji_motor
{
    DJI_Motor_Type_t motor_type;
    uint8_t can_bus;
    uint8_t speed_controller_id;

    /* Motor Config */
    uint8_t control_mode;
    uint8_t pos_feedback_absolute_angle;
    Motor_Reversal_t motor_reversal;
    uint8_t disabled;
    DJI_Motor_Stats_t *stats;

    /* External Sensor*/
    // external sensor information like imu or external encoders
    uint8_t use_external_feedback;         // 0 for no, 1 for yes
    int8_t external_feedback_dir;          // 0 for no, 1 for yes
    float *external_angle_feedback_ptr;    // pointer to the external angle feedback
    float *external_velocity_feedback_ptr; // pointer to the external velocity feedback

    /* Motor Controller */
    PID_t *angle_pid;
    PID_t *velocity_pid;
    PID_t *torque_pid;

    int16_t output_current;
} DJI_Motor_Handle_t;

typedef enum
{
    UP_UP_DOWN = 1,
    UP_UP_UP
} GM6020_CAN_ID_e;

typedef enum
{
    HEAD = 1,
    TAIL = 0
} DJI_Send_Type_e;

typedef struct _DJI_Send_Group_s
{
    uint8_t register_device_indicator;
    CAN_Instance_t *can_instance;
    int16_t *motor_torq[4];
} DJI_Send_Group_t;

/**
 * @brief  DJI Motor Initialization Function. Initialize the motor with the given configuration.
 * Check usage in documentation
 */
DJI_Motor_Handle_t *DJI_Motor_Init(Motor_Config_t *config, DJI_Motor_Type_t type);

/**
 * @brief  DJI Motor Initialization Function. Initialize the motor with the given configuration.
 * Check usage in documentation
 */
void DJI_Motor_Send(void);

/**
 * @brief Set the target motor torque
 *
 * @param motor the motor handle of the motor to control
 * @param torque the target torque
 */
void DJI_Motor_Set_Torque(DJI_Motor_Handle_t *motor, float torque);

/**      
 * @brief Set the target motor velocity
 *
 * @param motor the motor handle of the motor to control
 * @param velocity the target velocity (in rpm after gear reduction)
 */
void DJI_Motor_Set_Velocity(DJI_Motor_Handle_t *motor, float velocity);

/**
 * @brief Set the target angle for the motor
 *
 * @param motor the motor handle of the motor to control
 * @param angle the target angle
 */
void DJI_Motor_Set_Angle(DJI_Motor_Handle_t *motor, float angle);

/**
 * @brief Set the control mode for the motor
 *
 * @param motor the motor handle of the motor to control
 * @param control_mode the control mode: VELOCITY_CONTROL, POSITION_CONTROL, TORQUE_CONTROL
 */
void DJI_Motor_Set_Control_Mode(DJI_Motor_Handle_t *motor, uint8_t control_mode);

/**
 * @brief  Get currently velocity in the configured task space (right hand rule)
 */
float DJI_Motor_Get_Velocity(DJI_Motor_Handle_t *motor);

/**
 * @brief  Get currently angle in the configured task space (right hand rule)
 */
float DJI_Motor_Get_Absolute_Angle(DJI_Motor_Handle_t *motor);

/**
 * @brief  Get currently angle in the configured task space (right hand rule)
 */
float DJI_Motor_Get_Total_Angle(DJI_Motor_Handle_t *motor);

/**
 * @brief Disable motor, this would set flag to disable the motor
 * When sending can data, a zero current will be sent
 */
void DJI_Motor_Disable(DJI_Motor_Handle_t *motor);

/**
 * @brief Disable all DJI motors
 */
void DJI_Motor_Disable_All();

/**
 * @brief Enable all DJI motors
 *
 */
void DJI_Motor_Enable_All();

/**
 * @brief Check if motor is at reference angle
 */
uint8_t DJI_Motor_Is_At_Angle(DJI_Motor_Handle_t *motor_handle, float tolerance);

/**
 * @brief Check if motor is at reference velocity
 */
uint8_t DJI_Motor_Is_At_Velocity(DJI_Motor_Handle_t *motor_handle, float tolerance);

/**
 * @brief Check if motor is at reference torque
 */
uint8_t DJI_Motor_Is_At_Torque(DJI_Motor_Handle_t *motor_handle, float tolerance);

#endif
