#include "dm_motor.h"
#include "user_math.h"
#include <stdlib.h>
#include <string.h>

#define DM_MAX_DEVICE (10)
DM_Motor_Handle_t *g_dm_motors[DM_MAX_DEVICE] = {NULL};
uint8_t g_dm_motor_num = 0;

/**
 * int float_to_uint(float x, float x_min, float x_max, int bits)
 *
 * convert a float into int by spanning the range of the variable and
 * linearly assign the value
 *
 * Input:
 * float x: the float variable to be converted into int
 * float x_min: the min value of x
 * float x_max: the max value of x
 * int bits: bit number of converted int
 *
 * Output:
 * int: the converted int
 */
int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

static float DM_Motor_Unwrap_Position(DM_Motor_Stats_t* stats, float curr_pos, float wrap_period){
    if(!stats->position_initialized){
        stats->prev_pos_raw = curr_pos;
        stats->pos_unwrapped = curr_pos;
        stats->position_initialized = 1;

        return stats->pos_unwrapped;
    }

    float delta = curr_pos - stats->prev_pos_raw;
    float half_period = wrap_period * 0.5f;

    if(delta > half_period){
        delta -= wrap_period;
    }
    else if(delta < -half_period){
        delta += wrap_period;
    }

    stats->pos_unwrapped += delta;
    stats->prev_pos_raw = curr_pos;

    return stats->pos_unwrapped;
}

void DM_Motor_Decode(CAN_Instance_t *motor_can_instance)
{
    uint8_t *data = motor_can_instance->rx_buffer;
    DM_Motor_Stats_t *data_frame = (DM_Motor_Stats_t *)motor_can_instance->binding_motor_stats;

    data_frame->id = (data[0]) & 0x0F;
    data_frame->state = (data[0]) >> 4;
    data_frame->pos_int = (data[1] << 8) | data[2];
    data_frame->vel_int = (data[3] << 4) | (data[4] >> 4);
    data_frame->torq_int = ((data[4] & 0xF) << 8) | data[5];

    float direction = data_frame->motor_reversal == CW_POS ? -1.0f : 1.0f;
    float ratio = data_frame->gear_ratio;
    data_frame->pos_raw = uint_to_float(data_frame->pos_int, P_MIN, P_MAX, 16);
    float vel = uint_to_float(data_frame->vel_int, V_MIN, V_MAX, 12);
    data_frame->vel = direction * vel * ratio;
    float torq = uint_to_float(data_frame->torq_int, T_MIN, T_MAX, 12);
    
    data_frame->torq = direction * torq / ratio;
    data_frame->t_mos = (float)(data[6]);
    data_frame->t_rotor = (float)(data[7]);

    float unwrapped = DM_Motor_Unwrap_Position(data_frame, data_frame->pos_raw, P_MAX - P_MIN);
    data_frame->pos = direction * unwrapped * ratio + data_frame->pos_offset;
    __MAP_ANGLE_TO_UNIT_CIRCLE(data_frame->pos);
}

/**
 * @brief This function is the user interface to enable the motor
 * 
 * Since damiao motors are disabled when powered on, and there is a change that
 * a singular enable signal sent to the motor is interpreted and not received 
 * by the motor, we implement the following logic to ensure that the motor is enabled:
 * 
 * 1. The user calls this function to enable the motor
 * 2. The function sets the enable pending bit in send_pending_flag to 1
 * 3. In motor_task, stm32 sends the motor a control signal (MIT, Position, Position + Velocity)
 * 4. The motor sends a feedback signal to the stm32, and stm32 can read the error code
 * 5. If the motor enable status does not match the expected status, a pending flag is 
 *      set to send enable signal again, this logic is implmemented in motor control functions
 *      such as @ref DM_Motor_Ctrl_MIT()
 * 
 * The function that sends the physical CAN signal is @ref void DM_Motor_Send()
 */
void DM_Motor_Enable_Motor(DM_Motor_Handle_t *motor)
{
    uint8_t enable_needed = motor->enabled == 0 || motor->stats->state != DM_MOTOR_ENABLED;
    // set enable flag, this is the user intention to enable the motor, not neccessarily reflecting the motor status
    motor->enabled = 1;
    // set the flag to send the data
    if (enable_needed)
    {
        motor->send_pending_flag |= DM_MOTOR_ENABLE_PENDING;
    }
    // set disable flag to 0
    motor->send_pending_flag &= ~DM_MOTOR_DISABLE_PENDING;
}

/**
 * @brief This function is called by @ref DM_Motor_Send() to frame the CAN message before 
 * sending the enable signal to the motor
 */
void DM_Motor_Frame_Enable_Protocol(DM_Motor_Handle_t *motor)
{
    CAN_Instance_t *motor_can_instance = motor->can_instance;
    uint8_t *data = motor_can_instance->tx_buffer;

    motor_can_instance->tx_header->StdId = motor->tx_id;
    motor_can_instance->tx_header->DLC = 8;

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFC;
}

void DM_Motor_Disable_Motor(DM_Motor_Handle_t *motor)
{
    motor->enabled = 0;
    motor->send_pending_flag &= ~DM_MOTOR_ENABLE_PENDING;

    switch (motor->disable_behavior)
    {
    case DM_MOTOR_ZERO_CURRENT: 
    {
        motor->send_pending_flag |= DM_MOTOR_DISABLE_PENDING;
        break;
    }
    case DM_MOTOR_HARDWARE_DISABLE:
        motor->send_pending_flag |= DM_MOTOR_DISABLE_PENDING;
        break;
    default:
        break;
    }

}

void DM_Motor_Frame_Disable_Protocol(DM_Motor_Handle_t *motor)
{
    uint8_t *data = motor->can_instance->tx_buffer;

    data[0] = 0xFF;
    data[1] = 0xFF;
    data[2] = 0xFF;
    data[3] = 0xFF;
    data[4] = 0xFF;
    data[5] = 0xFF;
    data[6] = 0xFF;
    data[7] = 0xFD;
}

void DM_Motor_Frame_Zero_Current_Protocol(DM_Motor_Handle_t *motor)
{
    motor->can_instance->tx_header->StdId = motor->tx_id;
    motor->can_instance->tx_header->DLC = 8;

    uint8_t *data = motor->can_instance->tx_buffer;

    uint16_t pos_temp, vel_temp, kp_temp, kd_temp, torq_temp;

    pos_temp = float_to_uint(0, P_MIN, P_MAX, 16);
    vel_temp = float_to_uint(0, V_MIN, V_MAX, 12);
    kp_temp = float_to_uint(0, KP_MIN, KP_MAX, 12);
    kd_temp = float_to_uint(0, KD_MIN, KD_MAX, 12);
    torq_temp = float_to_uint(0, T_MIN, T_MAX, 12);

    data[0] = (pos_temp >> 8);
    data[1] = pos_temp;
    data[2] = (vel_temp >> 4);
    data[3] = ((vel_temp & 0xF) << 4) | (kp_temp >> 8);
    data[4] = kp_temp;
    data[5] = (kd_temp >> 4);
    data[6] = ((kd_temp & 0xF) << 4) | (torq_temp >> 8);
    data[7] = torq_temp;
}

void DM_Motor_Disable_All()
{
    for (int i = 0; i < g_dm_motor_num; i++)
    {
        if (g_dm_motors[i] == NULL)
        {
            continue;
        }
        DM_Motor_Disable_Motor(g_dm_motors[i]);
    }
}

void DM_Motor_Set_Control_Mode(DM_Motor_Handle_t *motor, uint8_t control_mode)
{
    if (control_mode > DM_MOTOR_VEL)
    {
        return;
    }
    motor->control_mode = control_mode;
    // Initialization runs before CAN_Service_Init(). Queue the write for the motor task.
    motor->send_pending_flag |= DM_MOTOR_MODE_PENDING;
}

static HAL_StatusTypeDef DM_Motor_Send_Control_Mode(DM_Motor_Handle_t *motor)
{
    // Use a separate frame so changing mode cannot overwrite a queued setpoint.
    CAN_Instance_t request = {0};
    CAN_TxHeaderTypeDef header = {0};
    request.can_bus = motor->can_bus;
    request.tx_header = &header;
    header.StdId = 0x7FF;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = 8;

    // The public constants are zero-based; CTRL_MODE is 1=MIT, 2=POS_VEL, 3=VEL.
    uint8_t *data = request.tx_buffer;
    data[0] = motor->tx_id & 0xFF;
    data[1] = (motor->tx_id >> 8) & 0xFF;
    data[2] = 0x55; // Write register to RAM (no Flash write)
    data[3] = 0x0A; // CTRL_MODE
    data[4] = motor->control_mode + 1U;
    return CAN_Transmit(&request);
}

void DM_Motor_Ctrl_MIT(DM_Motor_Handle_t *motor, float target_pos, float target_vel, float torq)
{
    uint16_t pos_temp, vel_temp, kp_temp, kd_temp, torq_temp;
    CAN_Instance_t *motor_can_instance = motor->can_instance;
    uint8_t *data = motor_can_instance->tx_buffer;
    float direction = motor->stats->motor_reversal == CW_POS ? -1.0f : 1.0f;
    float ratio = motor->stats->gear_ratio;
    motor->target_pos = direction * (target_pos - motor->stats->pos_offset) / ratio;
    motor->target_vel = direction * target_vel / ratio;
    motor->torq = direction * torq * ratio;

    __MAX_LIMIT(motor->target_pos, P_MIN, P_MAX);
    __MAX_LIMIT(motor->target_vel, V_MIN, V_MAX);
    __MAX_LIMIT(motor->torq, T_MIN, T_MAX);
    pos_temp = float_to_uint(motor->target_pos, P_MIN, P_MAX, 16);
    vel_temp = float_to_uint(motor->target_vel, V_MIN, V_MAX, 12);
    kp_temp = float_to_uint(motor->kp, KP_MIN, KP_MAX, 12);
    kd_temp = float_to_uint(motor->kd, KD_MIN, KD_MAX, 12);
    torq_temp = float_to_uint(motor->torq, T_MIN, T_MAX, 12);

    data[0] = (pos_temp >> 8);
    data[1] = pos_temp;
    data[2] = (vel_temp >> 4);
    data[3] = ((vel_temp & 0xF) << 4) | (kp_temp >> 8);
    data[4] = kp_temp;
    data[5] = (kd_temp >> 4);
    data[6] = ((kd_temp & 0xF) << 4) | (torq_temp >> 8);
    data[7] = torq_temp;

    // set the flag to send the data
    motor->send_pending_flag |= DM_MOTOR_SEND_PENDING;
    if (motor->enabled == 1 && motor->stats->state != DM_MOTOR_ENABLED)
    {
        motor->send_pending_flag |= DM_MOTOR_ENABLE_PENDING; // set the enable pending flag
    }
}

void DM_Motor_Ctrl_MIT_PD(DM_Motor_Handle_t *motor, float target_pos, float target_vel, float torq, float kp, float kd)
{
    uint16_t pos_temp, vel_temp, kp_temp, kd_temp, torq_temp;
    CAN_Instance_t *motor_can_instance = motor->can_instance;
    uint8_t *data = motor_can_instance->tx_buffer;

    float direction = motor->stats->motor_reversal == CW_POS ? -1.0f : 1.0f;
    float ratio = motor->stats->gear_ratio;
    motor->target_pos = direction * (target_pos - motor->stats->pos_offset) / ratio;
    motor->target_vel = direction * target_vel / ratio;
    motor->torq = direction * torq * ratio;

    __MAX_LIMIT(motor->target_pos, P_MIN, P_MAX);
    __MAX_LIMIT(motor->target_vel, V_MIN, V_MAX);
    __MAX_LIMIT(motor->torq, T_MIN, T_MAX);

    pos_temp = float_to_uint(motor->target_pos, P_MIN, P_MAX, 16);
    vel_temp = float_to_uint(motor->target_vel, V_MIN, V_MAX, 12);
    kp_temp = float_to_uint(kp, KP_MIN, KP_MAX, 12);
    kd_temp = float_to_uint(kd, KD_MIN, KD_MAX, 12);
    torq_temp = float_to_uint(motor->torq, T_MIN, T_MAX, 12);

    data[0] = (pos_temp >> 8);
    data[1] = pos_temp;
    data[2] = (vel_temp >> 4);
    data[3] = ((vel_temp & 0xF) << 4) | (kp_temp >> 8);
    data[4] = kp_temp;
    data[5] = (kd_temp >> 4);
    data[6] = ((kd_temp & 0xF) << 4) | (torq_temp >> 8);
    data[7] = torq_temp;

    // set the flag to send the data
    motor->send_pending_flag |= DM_MOTOR_SEND_PENDING;
    if (motor->enabled == 1 && motor->stats->state != DM_MOTOR_ENABLED)
    {
        motor->send_pending_flag |= DM_MOTOR_ENABLE_PENDING; // set the enable pending flag
    }
}

void DM_Motor_Set_MIT_PD(DM_Motor_Handle_t *motor, float kp, float kd)
{
    motor->kp = kp;
    motor->kd = kd;
}

DM_Motor_Handle_t *DM_Motor_Init(DM_Motor_Config_t *config)
{
    DM_Motor_Handle_t *motor = malloc(sizeof(DM_Motor_Handle_t));
    motor->can_bus = config->can_bus;
    motor->control_mode = config->control_mode;
    motor->enabled = 0;
    motor->tx_id = config->tx_id;
    motor->rx_id = config->rx_id;
    motor->disable_behavior = config->disable_behavior; // by defualt set to zero current

    motor->send_pending_flag = 0;
    motor->target_vel = 0.0f;
    motor->target_pos = 0.0f;
    motor->torq = 0.0f;

    motor->kp = config->kp;
    motor->kd = config->kd;
    motor->stats = calloc(sizeof(DM_Motor_Stats_t), 1);
    motor->stats->pos_offset = config->pos_offset;
    motor->stats->gear_ratio = config->gear_ratio != 0 ? config->gear_ratio : 1.0f;
    motor->stats->motor_reversal = config->motor_reversal;

    motor->can_instance = CAN_Device_Register(motor->can_bus, motor->tx_id, motor->rx_id, DM_Motor_Decode);
    motor->can_instance->binding_motor_stats = (void *)motor->stats;

    g_dm_motors[g_dm_motor_num++] = motor;

    DM_Motor_Set_Control_Mode(motor, motor->control_mode);

    return motor;
}

void DM_Motor_CtrlPosVel(DM_Motor_Handle_t *motor, float target_pos, float target_vel)
{
    uint8_t *data = motor->can_instance->tx_buffer;
    // motor->target_pos = target_pos;
    target_pos = (motor->stats->motor_reversal==CW_POS ? -1 : 1) * (target_pos - motor->stats->pos_offset) / motor->stats->gear_ratio;
    motor->target_pos = target_pos;

    target_vel = target_vel / motor->stats->gear_ratio;
    motor->target_vel = target_vel;

    // STM32 uses little-endian floats, matching the DaMiao POS_VEL payload.
    memcpy(data, &target_pos, sizeof(target_pos));
    memcpy(data + sizeof(target_pos), &target_vel, sizeof(target_vel));

    motor->send_pending_flag |= DM_MOTOR_SEND_PENDING;
    if (motor->enabled == 1 && motor->stats->state != DM_MOTOR_ENABLED)
    {
        motor->send_pending_flag |= DM_MOTOR_ENABLE_PENDING;
    }
}

void DM_Motor_CtrlVel(DM_Motor_Handle_t *motor, float target_vel)
{   
    CAN_Instance_t *motor_can_instance = motor->can_instance;
    uint8_t *data = motor_can_instance->tx_buffer;
    target_vel = motor->stats->motor_reversal == CCW_POS ? target_vel : -target_vel;
    target_vel /= motor->stats->gear_ratio;
    motor->target_vel = target_vel;

    uint8_t *vel_bytes = (uint8_t *) &target_vel;

    data[0] = vel_bytes[0];
    data[1] = vel_bytes[1];
    data[2] = vel_bytes[2];
    data[3] = vel_bytes[3];

    motor->send_pending_flag |= DM_MOTOR_SEND_PENDING;
    if (motor->enabled == 1 && motor->stats->state != DM_MOTOR_ENABLED)
    {
        motor->send_pending_flag |= DM_MOTOR_ENABLE_PENDING; // set the enable pending flag
    }

}

void DM_Motor_Send()
{
    for (int i = 0; i < g_dm_motor_num; i++) // loop through all the motors
    {
        DM_Motor_Handle_t *motor = g_dm_motors[i];                                    // check if the flag is set
        if (motor->send_pending_flag & DM_MOTOR_MODE_PENDING)
        {
            if (DM_Motor_Send_Control_Mode(motor) == HAL_OK)
            {
                motor->send_pending_flag &= ~DM_MOTOR_MODE_PENDING;
            }
            // Give the mode write a separate iteration before enable or control frames.
            continue;
        }
        if (motor->send_pending_flag & DM_MOTOR_SEND_PENDING)
        {           
            switch(motor->control_mode){
                case DM_MOTOR_MIT:
                    motor->can_instance->tx_header->StdId = motor->tx_id;
                    motor->can_instance->tx_header->DLC = 8;
                    break;
                case DM_MOTOR_POS_VEL:
                    motor->can_instance->tx_header->StdId = motor->tx_id + 0x100;
                    motor->can_instance->tx_header->DLC = 8;
                    break;
                case DM_MOTOR_VEL:
                    motor->can_instance->tx_header->StdId = motor->tx_id + 0x200;
                    motor->can_instance->tx_header->DLC = 4;
                    break;
                default:
                    break;
            }
            if (CAN_Transmit(motor->can_instance) == HAL_OK)
            {
                motor->send_pending_flag &= ~DM_MOTOR_SEND_PENDING;
            }
        }
        if (g_dm_motors[i]->send_pending_flag & DM_MOTOR_ENABLE_PENDING) 
        // Check if enable pending, this flag is set when the motor is enabled by user but motor feedback shows otherwise
        {
            // Special frames must not overwrite a setpoint queued by the robot task.
            CAN_Instance_t request = {0};
            CAN_TxHeaderTypeDef header = {0};
            DM_Motor_Handle_t command = *motor;
            request.can_bus = motor->can_bus;
            request.tx_header = &header;
            header.IDE = CAN_ID_STD;
            header.RTR = CAN_RTR_DATA;
            command.can_instance = &request;
            DM_Motor_Frame_Enable_Protocol(&command);
            if (CAN_Transmit(&request) == HAL_OK)
            {
                motor->send_pending_flag &= ~DM_MOTOR_ENABLE_PENDING;
            }
        }
        if (g_dm_motors[i]->send_pending_flag & DM_MOTOR_DISABLE_PENDING)
        {
            CAN_Instance_t request = {0};
            CAN_TxHeaderTypeDef header = {0};
            DM_Motor_Handle_t command = *motor;
            request.can_bus = motor->can_bus;
            request.tx_header = &header;
            header.StdId = motor->tx_id;
            header.DLC = 8;
            header.IDE = CAN_ID_STD;
            header.RTR = CAN_RTR_DATA;
            command.can_instance = &request;
            DM_Motor_Frame_Disable_Protocol(&command);
            CAN_Transmit(&request);
            DM_Motor_Frame_Zero_Current_Protocol(&command);
            CAN_Transmit(&request);
            // does not clear the flag, this is to ensure to keep sending so the motor is disabled
        }
    }
}
