#include "gimbal_task.h"

#include "robot.h"
#include "remote.h"
#include "user_math.h"
#include "dji_motor.h"
#include "imu_task.h"
#include "jetson_orin.h"
#include "dm_motor.h"

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;
DJI_Motor_Handle_t *g_yaw;
DM_Motor_Handle_t *g_pitch;

void Gimbal_Task_Init()
{
    Motor_Config_t yaw_motor_config = {
        .can_bus = 1,
        .speed_controller_id = 3, // probably set this to 5 someday
        .offset = YAW_OFFSET,
        .control_mode = POSITION_VELOCITY_SERIES,
        .use_external_feedback = 1,
        .external_feedback_dir = 1,
        .external_angle_feedback_ptr = &g_imu.rad.yaw,
        .external_velocity_feedback_ptr = &(g_imu.bmi088_raw.gyro[2]),
        .motor_reversal = MOTOR_REVERSAL_REVERSED,
        .angle_pid =
            {
                .kp = 300.0f,
                .kd = 30.0f,
                .output_limit = GM6020_MAX_CURRENT,
            },
        .velocity_pid =
            {
                .kp = 100.0f,
                .ki = 0.0f,
                .kf = 100.0f,
                .feedforward_limit = 5000.0f,
                .integral_limit = 1000.0f,
                .output_limit = GM6020_MAX_CURRENT,
            },
    };

    DM_Motor_Config_t pitch_motor_config = {
        .can_bus = 2,
        .control_mode = DM_MOTOR_MIT,
        .rx_id = 0x51,
        .tx_id = 0x01,
        .disable_behavior = DM_MOTOR_ZERO_CURRENT,
        .kp = 10.0f,
        .kd = 1.0f,
    };

    g_yaw = DJI_Motor_Init(&yaw_motor_config, GM6020);
    g_pitch = DM_Motor_Init(&pitch_motor_config);
}


void Gimbal_Ctrl_Loop()
{
    g_robot_state.gimbal.yaw_angle -= g_remote.controller.right_stick.x/660.0f * 0.01f;
    g_robot_state.gimbal.pitch_angle += g_remote.controller.right_stick.y / 660.0f * 0.01f;

    // hardware limits for gimbal pitch (prevent self collision)
    __MAP_ANGLE_TO_UNIT_CIRCLE(g_robot_state.gimbal.yaw_angle);
    // __MAX_LIMIT(g_robot_state.gimbal.pitch_angle, -0.4f, 0.4f);

    // Control loop for gimbal
    // DJI_Motor_Set_Angle(g_pitch, g_robot_state.gimbal.pitch_angle);
    DJI_Motor_Set_Angle(g_yaw, g_robot_state.gimbal.yaw_angle);
    // __MAX_LIMIT(g_robot_state.gimbal.pitch_angle, -0.45f, 0.4f);
    
    // DM_Motor_Enable_Motor(g_pitch);
    // DM_Motor_Ctrl_MIT_PD(g_pitch, g_robot_state.gimbal.pitch_angle, 0.0f, 0.0f, 20.0f, 8.5f);
}

void _Gimbal_Target_Reset()
{
    g_robot_state.gimbal.yaw_angle = g_imu.rad.yaw;
}

void Gimbal_Task_Disable()
{
    _Gimbal_Target_Reset();
}