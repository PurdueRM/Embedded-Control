#include "gimbal_task.h"

#include "robot.h"
#include "remote.h"
#include "user_math.h"
#include "dji_motor.h"
#include "imu_task.h"
#include "jetson_orin.h"
#include "dm_motor.h"
#include "pid.h"

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;
DJI_Motor_Handle_t *g_bottom_motor;
DM_Motor_Handle_t *g_top_yaw, *g_pitch;

PID_t g_top_yaw_pid = {
    .kp = 3.0f,
    .ki = 0.0f,
    .kd = 1000.0f,
    .integral_limit = 0.0f,
    .output_limit = 3.0f
};
PID_t g_bottom_yaw_follow_pid = {
    .kp = 50000.0f,
    .kd = 10000000.0f,
    .ki = 0.0f,
    .integral_limit = 0.0f,
    .output_limit = GM6020_MAX_VOLTAGE_INT,
};
float g_top_yaw_torque = 0.0f;
float g_gimbal_angle_difference = 0.0f;
void Gimbal_Task_Init()
{
    Motor_Config_t yaw_motor_config = {
        .can_bus = 1,
        .speed_controller_id = 3,
        .offset = 8016,
        .bypass_driver = true,
    };
    // Secret message
    DM_Motor_Config_t top_yaw_motor_config = {
        .can_bus = 2,
        .control_mode = DM_MOTOR_MIT,
        .pos_offset = -0.48f,
        .rx_id = 0x15,
        .tx_id = 0x05,
        .disable_behavior = DM_MOTOR_ZERO_CURRENT,
        .kp = 10.0f,
        .kd = 1.0f,
    };

    DM_Motor_Config_t pitch_motor_config = {
        .can_bus = 2,
        .control_mode = DM_MOTOR_MIT,
        .pos_offset = 0.0f,
        .rx_id = 0x11,
        .tx_id = 0x01,
        .disable_behavior = DM_MOTOR_ZERO_CURRENT,
        .kp = 10.0f,
        .kd = 1.0f,
    };

    g_top_yaw = DM_Motor_Init(&top_yaw_motor_config);
    g_pitch = DM_Motor_Init(&pitch_motor_config);
    g_bottom_motor = DJI_Motor_Init(&yaw_motor_config, GM6020);
}


void Gimbal_Ctrl_Loop()
{
    g_robot_state.gimbal.yaw_angle -= g_remote.controller.right_stick.x/660.0f * 0.01f + g_remote.mouse.x / 10000.0f;
    // g_robot_state.gimbal.pitch_angle += g_remote.controller.right_stick.y / 660.0f * 0.01f + g_remote.mouse.y / 50000.0f;
    g_gimbal_angle_difference = g_top_yaw->stats->pos;
    __MAP_ANGLE_TO_UNIT_CIRCLE(g_gimbal_angle_difference);
    __MAX_LIMIT(g_gimbal_angle_difference, -1.0f, 1.0f);
    g_bottom_motor->output_current = PID(&g_bottom_yaw_follow_pid, g_gimbal_angle_difference);
    // // hardware limits for gimbal yaw (prevent self collision)
    __MAP_ANGLE_TO_UNIT_CIRCLE(g_robot_state.gimbal.yaw_angle);
    g_top_yaw_torque = PID(&g_top_yaw_pid, g_robot_state.gimbal.yaw_angle - g_imu.rad.yaw);

    // __MAX_LIMIT(g_robot_state.gimbal.yaw_angle, -1.05f, 0.33f);
    
    DM_Motor_Enable_Motor(g_top_yaw);
    
    DM_Motor_Ctrl_MIT_PD(g_top_yaw, 0.0f, 0.0f, g_top_yaw_torque, 0.0f, 0.0f);



    g_robot_state.gimbal.pitch_angle -= g_remote.controller.right_stick.y / 660.0f * 0.01f;
    DM_Motor_Enable_Motor(g_pitch);
    DM_Motor_Ctrl_MIT_PD(g_pitch, g_robot_state.gimbal.pitch_angle, 0.0f, 0.0f, 20.0f, 8.5f);
}

void _Gimbal_Target_Reset()
{
    g_robot_state.gimbal.yaw_angle = g_imu.rad.yaw;
}

void Gimbal_Task_Disable()
{
    _Gimbal_Target_Reset();
    g_bottom_motor->output_current = 0;
    //ling gan guli guli
}