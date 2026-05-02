#include "gimbal_task.h"

#include "robot.h"
#include "remote.h"
#include "user_math.h"
#include "dji_motor.h"
#include "imu_task.h"
#include "jetson_orin.h"

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;

DM_Motor_Handle_t *g_wrist;
DM_Motor_Handle_t *g_shoulder;
DM_Motor_Handle_t *g_elbow;

float elbow_target;
float shoulder_target;
float wrist_target;

void Gimbal_Task_Init()
{
    DM_Motor_Config_t wrist_motor_config = {
        .can_bus = 1,
        .control_mode = DM_MOTOR_MIT,
        .rx_id = 0x02,  //Master ID
        .tx_id = 0x03,  //CAN ID 
        .disable_behavior = DM_MOTOR_ZERO_CURRENT,
        .pos_offset = -1.753,
        .kp = 10.0f,
        .kd = 1.0f,
    };
    DM_Motor_Config_t shoulder_motor_config = {
        .can_bus = 1,
        .control_mode = DM_MOTOR_MIT,
        .rx_id = 0x12,  //Master ID
        .tx_id = 0x13,  //CAN ID 
        .disable_behavior = DM_MOTOR_ZERO_CURRENT,
        .kp = 10.0f,
        .kd = 1.0f,
    };
    DM_Motor_Config_t elbow_motor_config = {
        .can_bus = 1,
        .control_mode = DM_MOTOR_MIT,
        .rx_id = 0x00,  //Master ID
        .tx_id = 0x01,  //CAN ID 
        .pos_offset = 0.6357,
        .disable_behavior = DM_MOTOR_ZERO_CURRENT,
        .kp = 10.0f,
        .kd = 1.0f,
    };

    g_wrist = DM_Motor_Init(&wrist_motor_config);
    g_shoulder = DM_Motor_Init(&shoulder_motor_config);
    g_elbow = DM_Motor_Init(&elbow_motor_config);

    shoulder_target = 0.0f;
    elbow_target = 0.0f;
    wrist_target = 0.0f;
}

void Gimbal_Ctrl_Loop()
{
    if(g_remote.controller.left_switch == 3){   //Mid
        elbow_target += g_remote.controller.left_stick.y/660.0f * 0.001;

        DM_Motor_Enable_Motor(g_elbow);
        DM_Motor_Ctrl_MIT_PD(g_elbow, elbow_target, 0.0f, 0.0f, 10.0f, 0.0f);

        wrist_target += g_remote.controller.right_stick.y/660.0f * 0.001;

        DM_Motor_Enable_Motor(g_wrist);
        DM_Motor_Ctrl_MIT_PD(g_wrist, wrist_target, 0.0f, 0.0f, 10.0f, 0.0f);
    }
    
}
