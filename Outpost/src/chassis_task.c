#include "chassis_task.h"

#include "robot.h"
#include "remote.h"
#include "dji_motor.h"
#include "motor.h"

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;

float chassis_rad;

DJI_Motor_Handle_t *g_yaw_motor;

void Chassis_Task_Init()
{
    // Init chassis hardware
    Motor_Config_t yaw_motor_config = {
        .can_bus = 1,
        .speed_controller_id = 1,
        .control_mode = VELOCITY_CONTROL,
        .motor_reversal = MOTOR_REVERSAL_NORMAL,
        .velocity_pid = {
            .kp = 500.0f,
            .kd = 200.0f,
            .kf = 100.0f,
            .output_limit = M3508_MAX_CURRENT_INT,
            .integral_limit = 3000.0f,
        }};
    g_yaw_motor = DJI_Motor_Init(&yaw_motor_config, M3508);
}

void Chassis_Ctrl_Loop()
{
    // Control loop for the chassis
    DJI_Motor_Set_Velocity(g_yaw_motor, 30.0f); // random value for testing
}