#include "gimbal_task.h"

#include "robot.h"
#include "remote.h"
#include "user_math.h"
#include "dji_motor.h"
#include "imu_task.h"
#include "jetson_orin.h"

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;
DJI_Motor_Handle_t *g_yaw;

void Gimbal_Task_Init()
{
    Motor_Config_t yaw_motor_config = {
        .can_bus = 1,
        .speed_controller_id = 1,
        .offset = 2400,
        .control_mode = POSITION_VELOCITY_SERIES,
        .motor_reversal = MOTOR_REVERSAL_NORMAL,
        .use_external_feedback = 1,
        .external_feedback_dir = 1,
        .external_angle_feedback_ptr = &g_imu.rad.yaw,
        .external_velocity_feedback_ptr = &(g_imu.bmi088_raw.gyro[2]),
        .angle_pid =
            {
                .kp = 25.0f,
                .kd = 10.0f,
                .output_limit = 25,
            },
        .velocity_pid =
            {
                .kp = 1000.0f,
                .ki = 0.0f,
                .kf = 0.0f,
                .feedforward_limit = 5000.0f,
                .integral_limit = 5000.0f,
                .output_limit = GM6020_MAX_CURRENT,
            },
    };
    g_yaw = DJI_Motor_Init(&yaw_motor_config, GM6020);
}


void Gimbal_Ctrl_Loop()
{
    if (g_robot_state.launch.IS_AUTO_AIMING_ENABLED) {
        if (g_orin_data.receiving.auto_aiming.yaw != 0 || g_orin_data.receiving.auto_aiming.pitch != 0)
        {
            float imu_yaw_delta = g_imu.rad.yaw + g_orin_data.receiving.auto_aiming.yaw / 180.0f * PI;
            float imu_pitch_delta = g_imu.rad.pitch + g_orin_data.receiving.auto_aiming.pitch / 180.0f * PI;
            __SLEW_RATE_LIMIT(g_robot_state.gimbal.yaw_angle, imu_yaw_delta, 0.2);
            __SLEW_RATE_LIMIT(g_robot_state.gimbal.pitch_angle, imu_pitch_delta, 0.2);
        }
    }

    // hardware limits for gimbal pitch (prevent self collision)
    g_robot_state.gimbal.yaw_angle = fmod(g_robot_state.gimbal.yaw_angle, 2 * PI);
    __MAX_LIMIT(g_robot_state.gimbal.pitch_angle, -0.4f, 0.4f);

    // Control loop for gimbal
    // DJI_Motor_Set_Angle(g_pitch, g_robot_state.gimbal.pitch_angle);
    DJI_Motor_Set_Angle(g_yaw, g_robot_state.gimbal.yaw_angle);
}
