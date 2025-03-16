#include "dji_motor.h"

#include <stdint.h>
#include <stdlib.h>
#include "bsp_can.h"
#include "user_math.h"
#include "motor.h"
#include <math.h>

DJI_Motor_Handle_t *g_dji_motors[MAX_DJI_MOTORS] = {NULL};
uint8_t g_dji_motor_count = 0;
float test_tmd;
DJI_Send_Group_t *g_dji_send_group[MAX_DJI_MOTOR_GROUPS] = {NULL};
uint8_t g_dji_motor_group_count = 0;

void DJI_Motor_Decode(CAN_Instance_t *can_instance);
void DJI_Set_Position(DJI_Motor_Handle_t *motor_handle, float pos);
void DJI_Set_Speed(DJI_Motor_Handle_t *motor_handle, float speed);
void DJI_Set_Torque(DJI_Motor_Handle_t *motor_handle, float torque);
void DJI_Motor_Disable(DJI_Motor_Handle_t *motor_handle);

uint8_t DJI_Motor_Assign_To_Group(DJI_Motor_Handle_t *motor_handle, uint16_t tx_id)
{
    uint8_t can_bus = motor_handle->can_bus;
    uint8_t real_id; // the order of which the motor will be send in the send group can frame
    // 1-4 are the first group, 5-8 are the second group
    // this real id will decide the order of te motor torque in the 8-byte can data frame
    if (motor_handle->speed_controller_id > 4)
    {
        real_id = motor_handle->speed_controller_id - 5;
    }
    else
    {
        real_id = motor_handle->speed_controller_id - 1;
    }
    // check if the motor belongs to a group that has been created
    for (int i = 0; i < g_dji_motor_group_count; i++)
    {
        // if the group is already created, add the motor to the group
        if ((g_dji_send_group[i]->can_instance->can_bus == can_bus) && (g_dji_send_group[i]->can_instance->tx_header->StdId == tx_id))
        {
            // change register indicator to include the new motor. This will be used in @ref DJI_Motor_Send()
            g_dji_send_group[i]->register_device_indicator |= (1 << real_id);
            // output current pointer of the motor
            // send group will check the value pointed by the pointer and send it at a predefined frequency
            g_dji_send_group[i]->motor_torq[real_id] = &motor_handle->output_current;
            return 0;
        }
    }
    // if reach here, create a new group
    DJI_Send_Group_t *new_group = (DJI_Send_Group_t *)malloc(sizeof(DJI_Send_Group_t));
    new_group->can_instance = calloc(sizeof(CAN_Instance_t), 1);
    new_group->can_instance->can_bus = can_bus;
    // allocate memory for tx_header
    new_group->can_instance->tx_header = malloc(sizeof(CAN_TxHeaderTypeDef));
    new_group->can_instance->tx_header->StdId = tx_id;      // set id (the actual id that will be send)
    new_group->can_instance->tx_header->IDE = CAN_ID_STD;   // standard id (check CAN documentation for more information)
    new_group->can_instance->tx_header->RTR = CAN_RTR_DATA; // data frame (check CAN documentation for more information)
    new_group->can_instance->tx_header->DLC = 8;            // 8 bytes of data (check CAN documentation for more information)
    // change register indicator to include the new motor. This will be used in @ref DJI_Motor_Send()
    new_group->register_device_indicator = (1 << real_id);
    // output current pointer of the motor
    // send group will check the value pointed by the pointer and send it at a predefined frequency
    new_group->motor_torq[real_id] = &motor_handle->output_current;
    motor_handle->output_current = 0;
    // assign new send group to the global array
    g_dji_send_group[g_dji_motor_group_count++] = new_group;
    return 1;
}

DJI_Motor_Handle_t *DJI_Motor_Init(Motor_Config_t *config, DJI_Motor_Type_t type)
{
    DJI_Motor_Handle_t *motor_handle = malloc(sizeof(DJI_Motor_Handle_t));
    // Initialize motor handle with the given configuration
    motor_handle->motor_type = type;
    motor_handle->can_bus = config->can_bus;
    motor_handle->speed_controller_id = config->speed_controller_id;
    motor_handle->control_mode = config->control_mode;
    motor_handle->pos_feedback_absolute_angle = config->pos_feedback_absolute_angle;
    motor_handle->motor_reversal = config->motor_reversal;
    motor_handle->use_external_feedback = config->use_external_feedback;
    motor_handle->external_feedback_dir = config->external_feedback_dir;
    motor_handle->external_angle_feedback_ptr = config->external_angle_feedback_ptr;
    motor_handle->external_velocity_feedback_ptr = config->external_velocity_feedback_ptr;

    // Initialize motor stats
    DJI_Motor_Stats_t *motor_stats = malloc(sizeof(DJI_Motor_Stats_t));
    motor_stats->encoder_offset = config->offset;
    motor_stats->total_round = 0;
    motor_stats->current_tick = 0;
    motor_stats->current_vel_rpm = 0;
    motor_stats->current_torq = 0;
    motor_stats->temp = 0;
    motor_handle->stats = motor_stats;
    motor_handle->output_current = 0;

    // Initialize PID controllers (bitwise AND to check if the control mode is enabled)
    if ((motor_handle->control_mode & VELOCITY_CONTROL) == VELOCITY_CONTROL)
    {
        motor_handle->velocity_pid = malloc(sizeof(PID_t));
        memcpy(motor_handle->velocity_pid, &config->velocity_pid, sizeof(PID_t));
    }
    if ((motor_handle->control_mode & POSITION_CONTROL) == POSITION_CONTROL)
    {
        motor_handle->angle_pid = malloc(sizeof(PID_t));
        memcpy(motor_handle->angle_pid, &config->angle_pid, sizeof(PID_t));
    }
    if ((motor_handle->control_mode & TORQUE_CONTROL) == TORQUE_CONTROL)
    {
        motor_handle->torque_pid = malloc(sizeof(PID_t));
        memcpy(motor_handle->torque_pid, &config->torque_pid, sizeof(PID_t));
    }

    CAN_Instance_t *receiver_can_instance = NULL;
    // initialize receive instance and send Group assignment based on motor type (Check DJI motor for more information)
    switch (motor_handle->motor_type)
    {
    case GM6020:
        receiver_can_instance = CAN_Device_Register(config->can_bus, config->tx_id,
                                                    0x204 + config->speed_controller_id, DJI_Motor_Decode);
        receiver_can_instance->binding_motor_stats = motor_stats;
        motor_stats->reduction_ratio = GM6020_REDUCTION_RATIO;
        switch (motor_handle->speed_controller_id)
        {
        case 1:
        case 2:
        case 3:
        case 4:
            DJI_Motor_Assign_To_Group(motor_handle, 0x1FF);
            break;
        case 5:
        case 6:
        case 7:
            DJI_Motor_Assign_To_Group(motor_handle, 0x2FF);
            break;
        default:
            // TODO: LOG ERROR
            break;
        }
        break;
    case M3508:
        receiver_can_instance = CAN_Device_Register(config->can_bus, config->tx_id,
                                                    0x200 + config->speed_controller_id, DJI_Motor_Decode);
        receiver_can_instance->binding_motor_stats = motor_stats;
        motor_stats->reduction_ratio = M3508_REDUCTION_RATIO;
        switch (motor_handle->speed_controller_id)
        {
        case 1:
        case 2:
        case 3:
        case 4:
            DJI_Motor_Assign_To_Group(motor_handle, 0x200);
            break;
        case 5:
        case 6:
        case 7:
        case 8:
            DJI_Motor_Assign_To_Group(motor_handle, 0x1FF);
            break;
        default:
            break;
        }
        break;
    case M2006:
        receiver_can_instance = CAN_Device_Register(config->can_bus, config->tx_id,
                                                    0x200 + config->speed_controller_id, DJI_Motor_Decode);
        receiver_can_instance->binding_motor_stats = motor_stats;
        motor_stats->reduction_ratio = M2006_REDUCTION_RATIO;
        switch (motor_handle->speed_controller_id)
        {
        case 1:
        case 2:
        case 3:
        case 4:
            DJI_Motor_Assign_To_Group(motor_handle, 0x200);
            break;
        case 5:
        case 6:
        case 7:
        case 8:
            DJI_Motor_Assign_To_Group(motor_handle, 0x1FF);
            break;
        default:
            break;
        }
        break;
    default:
        break;
    }
    g_dji_motors[g_dji_motor_count++] = motor_handle;
    return motor_handle;
}

// absolute angle of motor (does not account for gear reduction)
float DJI_Motor_Get_Absolute_Angle(DJI_Motor_Handle_t *motor_handle)
{
    switch (motor_handle->motor_reversal)
    {
    case MOTOR_REVERSAL_NORMAL:
        return motor_handle->stats->absolute_angle_rad;
        break;
    case MOTOR_REVERSAL_REVERSED:
        return -motor_handle->stats->absolute_angle_rad + 2 * PI;
        break;
    }
    return -1;
}

// total angle of motor (accounts for gear reduction)
float DJI_Motor_Get_Total_Angle(DJI_Motor_Handle_t *motor_handle)
{
    switch (motor_handle->motor_reversal)
    {
    case MOTOR_REVERSAL_NORMAL:
        return motor_handle->stats->total_angle_rad;
        break;
    case MOTOR_REVERSAL_REVERSED:
        return -motor_handle->stats->total_angle_rad;
        break;
    }
    return -1;
}

float DJI_Motor_Get_Velocity(DJI_Motor_Handle_t *motor_handle)
{
    switch (motor_handle->motor_reversal)
    {
    case MOTOR_REVERSAL_NORMAL:
        return motor_handle->stats->current_vel_rpm;
        break;
    case MOTOR_REVERSAL_REVERSED:
        return -motor_handle->stats->current_vel_rpm;
        break;
    }
    return -1;
}

void DJI_Motor_Set_Angle(DJI_Motor_Handle_t *motor_handle, float angle)
{
    motor_handle->disabled = 0;
    switch (motor_handle->motor_reversal)
    {
    case MOTOR_REVERSAL_NORMAL:
        motor_handle->angle_pid->ref = angle;
        break;
    case MOTOR_REVERSAL_REVERSED:
        motor_handle->angle_pid->ref = -angle + 2 * PI;
        break;
    default:
        break;
    }
}

void DJI_Motor_Set_Velocity(DJI_Motor_Handle_t *motor_handle, float velocity)
{
    motor_handle->disabled = 0;
    switch (motor_handle->motor_reversal)
    {
    case MOTOR_REVERSAL_NORMAL:
        motor_handle->velocity_pid->ref = velocity;
        break;
    case MOTOR_REVERSAL_REVERSED:
        motor_handle->velocity_pid->ref = -velocity;
        break;
    default:
        break;
    }
}

void DJI_Motor_Set_Torque(DJI_Motor_Handle_t *motor_handle, float torque)
{
    motor_handle->disabled = 0;
    motor_handle->torque_pid->ref = torque;
}

void DJI_Motor_Set_Control_Mode(DJI_Motor_Handle_t *motor_handle, uint8_t control_mode)
{
    motor_handle->control_mode = control_mode;
}

void DJI_Motor_Disable(DJI_Motor_Handle_t *motor_handle)
{
    motor_handle->disabled = 1;
}

void DJI_Motor_Current_Calc()
{
    for (int i = 0; i < g_dji_motor_count; i++)
    {
        DJI_Motor_Handle_t *motor = g_dji_motors[i];
        float angle_feedback = 0;
        float velocity_feedback = 0;
        // assign feedback depending on whether the motor uses an external sensor (like imu)
        switch (motor->use_external_feedback)
        {
        case 1:
            angle_feedback = motor->external_feedback_dir * ((float)*(motor->external_angle_feedback_ptr));
            velocity_feedback = motor->external_feedback_dir * ((float)*(motor->external_velocity_feedback_ptr));
            break;
        case 0:
            angle_feedback = motor->stats->absolute_angle_rad;
            velocity_feedback = motor->stats->current_vel_rpm;
            break;
        }
        // check disabled flag
        switch (motor->disabled)
        {
        case 1:
            motor->output_current = 0;
            break;
        case 0:
            // calculate motor current based on control mode, check documentation for more information
            switch (motor->control_mode)
            {
            case VELOCITY_CONTROL:
            {
                motor->output_current = PID(motor->velocity_pid, motor->velocity_pid->ref - velocity_feedback);
                break;
            }
            case POSITION_CONTROL:
            {
                float error = motor->angle_pid->ref - angle_feedback;
                __MAP_ANGLE_TO_UNIT_CIRCLE(error);
                motor->output_current = PID(motor->angle_pid, error);
                break;
            }
            case POSITION_CONTROL_TOTAL_ANGLE:
            {
                float error = motor->angle_pid->ref - motor->stats->total_angle_rad;
                motor->output_current = PID(motor->angle_pid, error);
                break;
            }
            case TORQUE_CONTROL:
            {
                break;
            }
            case POSITION_VELOCITY_SERIES:
            {
                // angle pid
                float error = motor->angle_pid->ref - angle_feedback;
                __MAP_ANGLE_TO_UNIT_CIRCLE(error);
                // angle pid output as velocity ref
                float output_vel = PID(motor->angle_pid, error);
                // velocity pid
                motor->output_current = PID(motor->velocity_pid, output_vel - velocity_feedback);
                break;
            }
            case POSITION_VELOCITY_PARALLEL:
            {
                // angle pid
                float error = motor->angle_pid->ref - angle_feedback;
                __MAP_ANGLE_TO_UNIT_CIRCLE(error); // wrap angle error to (-pi, pi)
                float angle_pid_output = PID(motor->angle_pid, error);
                // velocity pid
                float velocity_pid_output = PID(motor->velocity_pid, motor->velocity_pid->ref - velocity_feedback);
                // add parallel output
                motor->output_current = angle_pid_output + velocity_pid_output;
                break;
            }
            default:
                break;
            }
            break;
        default:
            break;
        }
    }
}

/**
 * Iteration through the registered send groups and send the motor torques.
 * Send groups are initialized in @ref DJI_Motor_Assign_To_Group(), which is called
 * during initialization in @ref DJI_Motor_Init()
 */
void DJI_Motor_Send()
{
    // Calculate motor current based on control mode for all motors
    DJI_Motor_Current_Calc();
    // Send motor torques (iterate through send groups)
    for (int i = 0; i < g_dji_motor_group_count; i++)
    {
        DJI_Send_Group_t *group = g_dji_send_group[i];
        uint8_t register_indicator = group->register_device_indicator;
        uint8_t *data = group->can_instance->tx_buffer;
        // register_indicator is a 4-bit number, each bit represents a motor
        // e.g. 0b1011 means motor 1, 2, 4 are registered, and motor 3 is not
        // & is bitwise AND, if the bit is 1, the corresponding motor torque is sent
        if (register_indicator & 0b0001)
        {
            static int16_t motor1 = 0;
            motor1 = (*(group->motor_torq[0]));
            data[0] = motor1 >> 8;
            data[1] = motor1;
        }
        if (register_indicator & 0b0010)
        {
            static int16_t motor2 = 0;
            motor2 = (*(group->motor_torq[1]));
            data[2] = motor2 >> 8;
            data[3] = motor2;
        }
        if (register_indicator & 0b0100)
        {
            static int16_t motor3 = 0;
            motor3 = (*(group->motor_torq[2]));
            data[4] = motor3 >> 8;
            data[5] = motor3;
        }
        if (register_indicator & 0b1000)
        {
            static int16_t motor4 = 0;
            motor4 = (*(group->motor_torq[3]));
            data[6] = motor4 >> 8;
            data[7] = motor4;
        }
        if (CAN_Transmit(group->can_instance) != HAL_OK)
        {
            // Log Error (Transmission is not successful)
        }
    }
}

/**
 * Decode CAN frame for DJI motor
 *
 * encoder range [0, 8191]
 * current range [-16384, 16384]
 * speed unit rmpm
 * temp unit degree celcius
 */
void DJI_Motor_Decode(CAN_Instance_t *can_instance)
{
    DJI_Motor_Stats_t *motor = (DJI_Motor_Stats_t *)can_instance->binding_motor_stats; // binding in @ref DJI_Motor_Init
    uint8_t *data = can_instance->rx_buffer;

    /* CAN Frame Process*/
    motor->last_tick = motor->current_tick;
    motor->current_tick = (data[0] << 8 | data[1]);
    motor->current_vel_rpm = (int16_t)((data[2] << 8 | data[3])) * motor->reduction_ratio;
    motor->current_torq = (int16_t)(data[4] << 8 | data[5]);
    motor->temp = data[6];

    motor->last_absolute_angle_rad = motor->absolute_angle_rad;
    motor->absolute_angle_rad = motor->current_tick - motor->encoder_offset;
    if (motor->absolute_angle_rad >= 8192)
    {
        motor->absolute_angle_rad -= 8192;
    }
    else if (motor->absolute_angle_rad < 0)
    {
        motor->absolute_angle_rad += 8192;
    }
    /* absolute angle */
    __MAP(motor->absolute_angle_rad, 0, 8192, 0, 2 * PI);

    /* angle wrap */
    if (motor->absolute_angle_rad - motor->last_absolute_angle_rad > PI)
    {
        motor->total_round--;
    }
    else if (motor->absolute_angle_rad - motor->last_absolute_angle_rad < -PI)
    {
        motor->total_round++;
    }
#pragma message "there are some problem with total_angle_rad"
    motor->total_angle_rad = ((motor->total_round) * 2 * PI + motor->absolute_angle_rad) * motor->reduction_ratio;
}

void DJI_Motor_Disable_All()
{
    for (int i = 0; i < g_dji_motor_count; i++)
    {
        if (g_dji_motors[i] == NULL)
        {
            continue;
        }
        DJI_Motor_Disable(g_dji_motors[i]);
    }
}

void DJI_Motor_Enable_All()
{
    for (int i = 0; i < g_dji_motor_count; i++)
    {
        // set the motor state to enabled
        g_dji_motors[i]->disabled = 0;
    }
}

uint8_t DJI_Motor_Is_At_Angle(DJI_Motor_Handle_t *motor_handle, float tolerance) {
    if (fabsf(motor_handle->angle_pid->prev_error) < tolerance) {
        return 1;
    } else {
        return 0;
    }
}

uint8_t DJI_Motor_Is_At_Velocity(DJI_Motor_Handle_t *motor_handle, float tolerance) {
    if (fabsf(motor_handle->velocity_pid->prev_error) < tolerance)
    {
        return 1;
    } else {
        return 0;
    }
}

uint8_t DJI_Motor_Is_At_Torque(DJI_Motor_Handle_t *motor_handle, float tolerance) {
    if (fabsf(motor_handle->torque_pid->prev_error) < tolerance) {
        return 1;
    } else {
        return 0;
    }
}
