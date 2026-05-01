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
DM_Motor_Handle_t *g_wrist;
DM_Motor_Handle_t *g_shoulder;
DM_Motor_Handle_t *g_elbow;

float elbow_target;
float shoulder_target;
float wrist_target;


uint16_t lost_target_yaw_counter = 0;
uint16_t initiate_scanning_counter = 0;
Arm_Move_State arm_state = SHOULDER;

void Gimbal_Task_Init()
{
    // Motor_Config_t yaw_motor_config = {
    //     .can_bus = 1,
    //     .speed_controller_id = 1,
    //     .offset = 6141,
    //     .control_mode = POSITION_CONTROL_SPINTOP,
    //     .motor_reversal = MOTOR_REVERSAL_NORMAL,
    //     .use_external_feedback = 1,
    //     .external_feedback_dir = 1,
    //     .external_angle_feedback_ptr = &g_imu.rad.yaw,
    //     .external_velocity_feedback_ptr = &(g_imu.bmi088_raw.gyro[2]),
    //     .angle_pid =
    //         {
    //             .kp = 45.0f,
    //             .kd = 2000.0f,
    //             .output_limit = 1000, // max vel
    //         },
    //     .velocity_pid =
    //         {
    //             .kp = 12000.0f,
    //             .ki = 0.0f,
    //             .kd = 5000.0f,
    //             .kf = 1000.0f,
    //             .feedforward_limit = 13000.0f,
    //             .integral_limit = 1000.0f,
    //             .output_limit = GM6020_MAX_CURRENT_INT,
    //         },
    // };

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
    // g_yaw = DJI_Motor_Init(&yaw_motor_config, GM6020);
    // g_pitch = DM_Motor_Init(&pitch_motor_config);
    g_wrist = DM_Motor_Init(&wrist_motor_config);
    g_shoulder = DM_Motor_Init(&shoulder_motor_config);
    g_elbow = DM_Motor_Init(&elbow_motor_config);

    shoulder_target = 0.0f;
    elbow_target = 0.0f;
    wrist_target = 0.0f;
}


void Gimbal_Ctrl_Loop()
{
    // // if (g_robot_state.launch.IS_AUTO_AIMING_ENABLED) {
    // //     if (g_orin_data.receiving.auto_aiming.yaw != 0 || g_orin_data.receiving.auto_aiming.pitch != 0)
    // //     {
    // //         float imu_yaw_delta = g_imu.rad.yaw + g_orin_data.receiving.auto_aiming.yaw / 180.0f * PI;
    // //         float imu_pitch_delta = g_imu.rad.pitch + g_orin_data.receiving.auto_aiming.pitch / 180.0f * PI;
    // //         __SLEW_RATE_LIMIT(g_robot_state.gimbal.yaw_angle, imu_yaw_delta, 0.2);
    // //         __SLEW_RATE_LIMIT(g_robot_state.gimbal.pitch_angle, imu_pitch_delta, 0.2);
    // //     }
    // // }
    // if (g_remote.controller.right_switch == UP)
    // {
    //     if (g_orin_data.receiving.auto_aiming.yaw == 0) {
    //         lost_target_yaw_counter++;
    //         if (initiate_scanning_counter < 10000) { // overflow
    //             initiate_scanning_counter++;
    //         }
    //     }
    //     if (lost_target_yaw_counter > 500 && initiate_scanning_counter > 750 && g_orin_data.nav_online_flag == 0) {
    //         g_robot_state.gimbal.yaw_angle += (2*PI/24);
    //         g_robot_state.gimbal.pitch_angle = -0.1f; // reset pitch to current pitch
    //         lost_target_yaw_counter = 0;
    //     }
    //     // g_robot_state.gimbal.yaw_angle += g_orin_data.receiving.navigation.yaw_angular_rate * 0.002f; // TODO: move to gimbal task
    //     // Handled in jetson_orin.c
    //     if (g_orin_data.new_data_flag == 1)
    //     {
    //         if (g_orin_data.receiving.auto_aiming.yaw == 0) { // no target detected

    //             ;
    //         } else {
    //             initiate_scanning_counter = 0;
    //             g_robot_state.gimbal.yaw_angle = g_imu.rad.yaw + g_orin_data.receiving.auto_aiming.yaw / 180.0f * PI;
    //             g_robot_state.gimbal.pitch_angle = g_imu.rad.pitch + g_orin_data.receiving.auto_aiming.pitch / 180.0f * PI;
    //         }
    //         g_orin_data.new_data_flag = 0;
    //     }
    // } else {
    //     g_robot_state.gimbal.yaw_angle -= g_remote.controller.right_stick.x/660.0f * 0.01f;
    //     g_robot_state.gimbal.pitch_angle += g_remote.controller.right_stick.y / 660.0f * 0.01f;
    // }

    // // hardware limits for gimbal pitch (prevent self collision)
    // __MAP_ANGLE_TO_UNIT_CIRCLE(g_robot_state.gimbal.yaw_angle);
    // // __MAX_LIMIT(g_robot_state.gimbal.pitch_angle, -0.4f, 0.4f);

    // // Control loop for gimbal
    // // DJI_Motor_Set_Angle(g_pitch, g_robot_state.gimbal.pitch_angle);
    // DJI_Motor_Set_Angle(g_yaw, g_robot_state.gimbal.yaw_angle);
    // __MAX_LIMIT(g_robot_state.gimbal.pitch_angle, -0.45f, 0.4f);
    
    // DM_Motor_Enable_Motor(g_pitch);
    // DM_Motor_Ctrl_MIT_PD(g_pitch, g_robot_state.gimbal.pitch_angle, 0.0f, 0.0f, 20.0f, 8.5f);
    // uint8_t l_switch = g_remote.controller.left_switch;
    // if(l_switch == 1){ // 1 up; 2 down; 3 mid
    //     arm_state = SHOULDER;
    // }
    // else if(l_switch == 3){
    //     arm_state = ELBOW;
    // }
    // else if(l_switch == 2){
    //     arm_state = WRIST;
    // }

    // switch(arm_state){
    //     case SHOULDER:

    //         break;
    //     case WRIST:
    //         DM_Motor_Ctrl_MIT_PD(g_wrist, 1.0f, 0.0f, 0.0f, 20.0f, 8.5f);
    //         break;
    //     default:
    //         break;
    // }
    elbow_target += g_remote.controller.left_stick.y/660.0f * 0.001;
    wrist_target += g_remote.controller.right_stick.y/660.0f * 0.001;

    DM_Motor_Enable_Motor(g_elbow);
    DM_Motor_Ctrl_MIT_PD(g_elbow, elbow_target, 0.0f, 0.0f, 10.0f, 0.0f);

    DM_Motor_Enable_Motor(g_wrist);
    DM_Motor_Ctrl_MIT_PD(g_wrist, wrist_target, 0.0f, 0.0f, 10.0f, 0.0f);
}

void _Gimbal_Target_Reset()
{
    g_robot_state.gimbal.yaw_angle = g_imu.rad.yaw;
}

void Gimbal_Task_Disable()
{
    _Gimbal_Target_Reset();
}