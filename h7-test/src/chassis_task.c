#include "chassis_task.h"

#include "dji_motor.h"
#include "motor.h"
#include "robot.h"
#include "remote.h"

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;

float chassis_rad;

DJI_Motor_Handle_t * g_test_motor;

void Chassis_Task_Init()
{
    // Init chassis hardware
    Motor_Config_t test_motor_config = {
        .can_bus = 1,
        .speed_controller_id = 1,
        .offset = 0,
        .control_mode = VELOCITY_CONTROL,
        .motor_reversal = MOTOR_REVERSAL_NORMAL,
        .velocity_pid =
            {
                .kp = 500.0f,
                .output_limit = M3508_MAX_CURRENT_INT,
            },
    };
    g_test_motor = DJI_Motor_Init(&test_motor_config, M3508);

}

void Chassis_Ctrl_Loop()
{
    DJI_Motor_Set_Velocity(g_test_motor, 10.0f);
    // Control loop for the chassis
}