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
extern IMU_t g_imu;
DM_Motor_Handle_t *g_yaw;
DM_Motor_Handle_t *g_pitch;
PID_t gimbal_imu_pid = {
    .kp = 1.5f,
    .ki = 0.0f,
    .kd = 1000.0f,
    .integral_limit = 0.0f,
    .output_limit = 3.0f
};
float tmp_yaw_angle_diff = 0.0f;
float g_yaw_torque = 0.0f;


void Gimbal_Task_Init(){
    DM_Motor_Config_t yaw_motor_config = {
        .can_bus = 1,
        .control_mode = DM_MOTOR_MIT,
        .pos_offset = -1.616,
        .rx_id = 0x11,
        .tx_id = 0x01,
        .disable_behavior = DM_MOTOR_ZERO_CURRENT,
        .kp = 10.0f,
        .kd = 1.0f,
    };

    DM_Motor_Config_t pitch_motor_config = {
        .can_bus = 2,
        .control_mode = DM_MOTOR_MIT,
        .rx_id = 0x12,
        .tx_id = 0x02,
        .pos_offset = 2.526,
        .disable_behavior = DM_MOTOR_ZERO_CURRENT,
        .kp = 10.0f,
        .kd = 1.0f,
    };

    g_yaw = DM_Motor_Init(&yaw_motor_config);
    g_pitch = DM_Motor_Init(&pitch_motor_config);
}

void Gimbal_Ctrl_Loop(){
    // Control loop for gimbal
    if(fabs(g_remote.controller.right_stick.x) > 5.0f)
        g_robot_state.gimbal.yaw_angle -= g_remote.controller.right_stick.x/660.0f * 0.01f;

    //Gimbal follow imu PID
    tmp_yaw_angle_diff = g_robot_state.gimbal.yaw_angle - g_imu.rad.yaw + 1.616;
    __MAP_ANGLE_TO_UNIT_CIRCLE(tmp_yaw_angle_diff);
    g_yaw_torque = PID(&gimbal_imu_pid, tmp_yaw_angle_diff);
    __MAX_LIMIT(g_yaw_torque, -1.5f, 1.5f)

    DM_Motor_Enable_Motor(g_yaw);
    DM_Motor_Ctrl_MIT_PD(g_yaw, 0.0f, 0.0f, g_yaw_torque, 0.0f, 0.0f);

    g_robot_state.gimbal.pitch_angle -= g_remote.controller.right_stick.y / 660.0f * 0.01f;
    __MAX_LIMIT(g_robot_state.gimbal.pitch_angle, -0.45f, 0.4f);
    
    DM_Motor_Enable_Motor(g_pitch);
    DM_Motor_Ctrl_MIT_PD(g_pitch, g_robot_state.gimbal.pitch_angle, 0.0f, 0.0f, 20.0f, 8.5f);


}