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
DM_Motor_Handle_t *g_libido;

void Gimbal_Task_Init()
{
    Motor_Config_t yaw_motor_config = {
        .can_bus = 1,
        .speed_controller_id = 1,
        .offset = 6141,
        .control_mode = POSITION_VELOCITY_SERIES,
        .motor_reversal = MOTOR_REVERSAL_NORMAL,
        .use_external_feedback = 1,
        .external_feedback_dir = 1,
        .external_angle_feedback_ptr = &g_imu.rad.yaw,
        .external_velocity_feedback_ptr = &(g_imu.bmi088_raw.gyro[2]),
        .angle_pid =
            {
                .kp = 25.0f,
                .kd = 100.0f,
                .output_limit = 20, // max vel
            },
        .velocity_pid =
            {
                .kp = 5000.0f,
                .ki = 0.0f,
                .kd = 0.0f,
                .kf = 0.0f,
                .feedforward_limit = 10000.0f,
                .integral_limit = 1000.0f,
                .output_limit = GM6020_MAX_CURRENT_INT,
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

    DM_Motor_Config_t libido_motor_config = {
        .can_bus = 2,
        .control_mode = DM_MOTOR_MIT,
        .pos_offset = 4.29636765,
        .rx_id = 0x17,
        .tx_id = 0x07,
        .disable_behavior = DM_MOTOR_ZERO_CURRENT,
        .kp = 10.0f,
        .kd = 1.0f,
    };

    g_yaw = DJI_Motor_Init(&yaw_motor_config, GM6020);
    g_pitch = DM_Motor_Init(&pitch_motor_config);
    g_libido = DM_Motor_Init(&libido_motor_config); //Makes the lidar erect 
}


void Gimbal_Ctrl_Loop()
{
    DM_Motor_Enable_Motor(g_libido);
    DM_Motor_Ctrl_MIT_PD(g_libido, 0.0f, 0.0f, 0.0f, 20.0f, 8.5f);

    // if (g_robot_state.launch.IS_AUTO_AIMING_ENABLED) {
    //     if (g_orin_data.receiving.auto_aiming.yaw != 0 || g_orin_data.receiving.auto_aiming.pitch != 0)
    //     {
    //         float imu_yaw_delta = g_imu.rad.yaw + g_orin_data.receiving.auto_aiming.yaw / 180.0f * PI;
    //         float imu_pitch_delta = g_imu.rad.pitch + g_orin_data.receiving.auto_aiming.pitch / 180.0f * PI;
    //         __SLEW_RATE_LIMIT(g_robot_state.gimbal.yaw_angle, imu_yaw_delta, 0.2);
    //         __SLEW_RATE_LIMIT(g_robot_state.gimbal.pitch_angle, imu_pitch_delta, 0.2);
    //     }
    // }
    if (g_remote.controller.right_switch == UP)
    {
        // g_robot_state.gimbal.yaw_angle += g_orin_data.receiving.navigation.yaw_angular_rate * 0.002f; // TODO: move to gimbal task
        // Handled in jetson_orin.c
        if (g_orin_data.new_data_flag == 1)
        {
            if (g_orin_data.receiving.auto_aiming.yaw == 0) { // no target detected
                ;
            } else {
                g_robot_state.gimbal.yaw_angle = g_imu.rad.yaw + g_orin_data.receiving.auto_aiming.yaw / 180.0f * PI;
                g_robot_state.gimbal.pitch_angle = g_imu.rad.pitch + g_orin_data.receiving.auto_aiming.pitch / 180.0f * PI;
            }
            g_orin_data.new_data_flag = 0;
            
        }
    } else {
        g_robot_state.gimbal.yaw_angle -= g_remote.controller.right_stick.x/660.0f * 0.01f;
        g_robot_state.gimbal.pitch_angle += g_remote.controller.right_stick.y / 660.0f * 0.01f;
    }

    // hardware limits for gimbal pitch (prevent self collision)
    __MAP_ANGLE_TO_UNIT_CIRCLE(g_robot_state.gimbal.yaw_angle);
    // __MAX_LIMIT(g_robot_state.gimbal.pitch_angle, -0.4f, 0.4f);

    // Control loop for gimbal
    // DJI_Motor_Set_Angle(g_pitch, g_robot_state.gimbal.pitch_angle);
    DJI_Motor_Set_Angle(g_yaw, g_robot_state.gimbal.yaw_angle);
    __MAX_LIMIT(g_robot_state.gimbal.pitch_angle, -0.45f, 0.4f);
    
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
}