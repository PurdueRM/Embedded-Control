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
                .kd = 2000.0f,
                .output_limit = 20, // max vel
            },
        .velocity_pid =
            {
                .kp = 3000.0f,
                .ki = 0.0f,
                .kd = 00.0f,
                .kf = 0.0f,
                .feedforward_limit = 10000.0f,
                .integral_limit = 1000.0f,
                .output_limit = GM6020_MAX_CURRENT_INT,
            },
    };

    g_yaw = DJI_Motor_Init(&yaw_motor_config, GM6020);
    
}


void Gimbal_Ctrl_Loop()
{
    if (!g_yaw->stats->is_initialized) {
        return;
    }
    if (g_remote.controller.right_switch == UP)
    {
        // g_robot_state.gimbal.yaw_angle += g_orin_data.receiving.navigation.yaw_angular_rate * 0.002f; // TODO: move to gimbal task
        // Handled in jetson_orin.c
        if (g_orin_data.new_data_flag == 1)
        {
            if (g_orin_data.receiving.auto_aiming.yaw != 0) { // target detected
                g_robot_state.gimbal.yaw_angle = g_imu.rad.yaw + g_orin_data.receiving.auto_aiming.yaw / 180.0f * PI;
            }
            g_orin_data.new_data_flag = 0;
            
        }
    } else {
        g_robot_state.gimbal.yaw_angle -= g_remote.controller.right_stick.x/660.0f * 0.015f;
    }

    // hardware limits for gimbal pitch (prevent self collision)
    __MAP_ANGLE_TO_UNIT_CIRCLE(g_robot_state.gimbal.yaw_angle);
    // __MAX_LIMIT(g_robot_state.gimbal.pitch_angle, -0.4f, 0.4f);

    // Control loop for gimbal
    DJI_Motor_Set_Angle(g_yaw, g_robot_state.gimbal.yaw_angle);
    __MAX_LIMIT(g_robot_state.gimbal.pitch_angle, -0.45f, 0.4f);
    
}

void _Gimbal_Target_Reset()
{
    g_robot_state.gimbal.yaw_angle = g_imu.rad.yaw;
}

void Gimbal_Task_Disable()
{
    _Gimbal_Target_Reset();
}