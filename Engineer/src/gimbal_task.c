#include "gimbal_task.h"

#include "robot.h"
#include "remote.h"
#include "user_math.h"
#include "dm_motor.h"

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;

DM_Motor_Handle_t * g_wrist_one;
float wrist_angle1;

void Gimbal_Task_Init()
{
    // Init Gimbal Hardware
    DM_Motor_Config_t wrist_motor_config = {
        .can_bus = 1,
        .control_mode = DM_MOTOR_MIT,
        .rx_id = 0x00,
        .tx_id = 0x01,
        .disable_behavior = DM_MOTOR_ZERO_CURRENT,
        .kp = 10.0f,
        .kd = 1.0f,
    };

    g_wrist_one = DM_Motor_Init(wrist_motor_config);
}

void Gimbal_Ctrl_Loop()
{
    // Control loop for gimbal
    wrist_angle1 = 0;
    DM_Motor_Ctrl_MIT_PD(g_wrist_one, wrist_angle1, 0.0f, 0.0f, 20.0f, 8.5f);
}
