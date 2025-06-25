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
DM_Motor_Handle_t *g_top_yaw;

void Gimbal_Task_Init()
{
    DM_Motor_Config_t top_yaw_motor_config = {
        .can_bus = 2,
        .control_mode = DM_MOTOR_MIT,
        .rx_id = 0x15,
        .tx_id = 0x05,
        .disable_behavior = DM_MOTOR_ZERO_CURRENT,
        .kp = 10.0f,
        .kd = 1.0f,
    };

    g_top_yaw = DM_Motor_Init(&top_yaw_motor_config);
}


void Gimbal_Ctrl_Loop()
{
    g_robot_state.gimbal.yaw_angle -= g_remote.controller.right_stick.x/660.0f * 0.01f + g_remote.mouse.x / 10000.0f;
    // g_robot_state.gimbal.pitch_angle += g_remote.controller.right_stick.y / 660.0f * 0.01f + g_remote.mouse.y / 50000.0f;

    // // hardware limits for gimbal yaw (prevent self collision)
    __MAP_ANGLE_TO_UNIT_CIRCLE(g_robot_state.gimbal.yaw_angle);
    __MAX_LIMIT(g_robot_state.gimbal.yaw_angle, -1.05f, 0.33f);
    
    DM_Motor_Enable_Motor(g_top_yaw);
    
    DM_Motor_Ctrl_MIT_PD(g_top_yaw, g_robot_state.gimbal.yaw_angle, 0.0f, 0.0f, 20.0f, 1.0f);
}

void _Gimbal_Target_Reset()
{
    g_robot_state.gimbal.yaw_angle = g_imu.rad.yaw;
}

void Gimbal_Task_Disable()
{
    _Gimbal_Target_Reset();
}