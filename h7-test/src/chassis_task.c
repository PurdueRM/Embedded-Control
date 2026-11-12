#include "chassis_task.h"

#include "dji_motor.h"
#include "motor.h"
#include "robot.h"
#include "remote.h"

#include <stdlib.h>

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;
DJI_Motor_Handle_t *motor_w1;
DJI_Motor_Handle_t *motor_w2;
DJI_Motor_Handle_t *motor_w3;
DJI_Motor_Handle_t *motor_w4;
struct Wheel_Velos wheel_velo_desired = {0};
float chassis_rad;

struct Input controller_in = {
    .x_velo = 0,
    .y_velo = 0,
    .ang_velo = 0,
};

// Helpers

float max(struct Wheel_Velos* velos)
{
    float max = abs(velos->wheel_one_speed);
    if (max < abs(velos->wheel_two_speed)) max = abs(velos->wheel_two_speed);
    if (max < abs(velos->wheel_three_speed)) max = abs(velos->wheel_three_speed);
    if (max < abs(velos->wheel_four_speed)) max = abs(velos->wheel_four_speed);
    return max;
}

void updateWheelVelocity(struct Wheel_Velos* velos, struct Input* input) {
    velos->wheel_one_speed = (1 / WHEEL_RADIUS) * (-SIN45 * input->x_velo + COS45 * input->y_velo + WHEEL_DIST * input->ang_velo);
    velos->wheel_two_speed = (1 / WHEEL_RADIUS) * (-COS45 * input->x_velo + -SIN45 * input->y_velo + WHEEL_DIST * input->ang_velo);
    velos->wheel_three_speed = (1 / WHEEL_RADIUS) * (SIN45 * input->x_velo + -COS45 * input->y_velo + WHEEL_DIST * input->ang_velo);
    velos->wheel_four_speed = (1 / WHEEL_RADIUS) * (COS45 * input->x_velo + SIN45 * input->y_velo + WHEEL_DIST * input->ang_velo);
}

void desaturate(struct Wheel_Velos* velos) {
    float highest_speed = max(velos);
    if (abs(highest_speed) > MAX_SPEED) {
        float ratio = MAX_SPEED / highest_speed;
        velos->wheel_one_speed = velos->wheel_one_speed * ratio;
        velos->wheel_two_speed = velos->wheel_two_speed * ratio;
        velos->wheel_three_speed = velos->wheel_three_speed * ratio;
        velos->wheel_four_speed = velos->wheel_four_speed * ratio;

    }
}

void Chassis_Task_Init()
{
    Motor_Config_t chassis_w1 = {
        .can_bus = 1, // change later 
        .speed_controller_id = 1, // change later based on can bus
        .offset = 0,
        .control_mode = VELOCITY_CONTROL,
        .motor_reversal = MOTOR_REVERSAL_NORMAL,
        .velocity_pid = 
            { // tune later
                .kp = 500.0f,
                .kd = 0.0f,
                .kf = 0.0f,
                .output_limit = M2006_MAX_CURRENT,
            },
    };
    motor_w1 = DJI_Motor_Init(&chassis_w1, M2006);

    Motor_Config_t chassis_w2 = {
        .can_bus = 1, // change later 
        .speed_controller_id = 2, // change later based on can bus
        .offset = 0,
        .control_mode = VELOCITY_CONTROL,
        .motor_reversal = MOTOR_REVERSAL_NORMAL,
        .velocity_pid = 
            { // tune later
                .kp = 500.0f,
                .kd = 0.0f,
                .kf = 0.0f,
                .output_limit = M2006_MAX_CURRENT,
            },
    };
    motor_w2 = DJI_Motor_Init(&chassis_w2, M2006);

    Motor_Config_t chassis_w3 = {
        .can_bus = 1, // change later 
        .speed_controller_id = 3, // change later based on can bus
        .offset = 0,
        .control_mode = VELOCITY_CONTROL,
        .motor_reversal = MOTOR_REVERSAL_NORMAL,
        .velocity_pid = 
            { // tune later
                .kp = 500.0f,
                .kd = 0.0f,
                .kf = 0.0f,
                .output_limit = M2006_MAX_CURRENT,
            },
    };
    motor_w3 = DJI_Motor_Init(&chassis_w3, M2006);

    Motor_Config_t chassis_w4 = {
        .can_bus = 1, // change later 
        .speed_controller_id = 4, // change later based on can bus
        .offset = 0,
        .control_mode = VELOCITY_CONTROL,
        .motor_reversal = MOTOR_REVERSAL_NORMAL,
        .velocity_pid = 
            { // tune later
                .kp = 500.0f,
                .kd = 0.0f,
                .kf = 0.0f,
                .output_limit = M2006_MAX_CURRENT,
            },
    };
    motor_w4 = DJI_Motor_Init(&chassis_w4, M2006);
}

void Chassis_Ctrl_Loop() // This is for the kinematics, everything else is handled in robot.c. Use Robot_State_t and Remote_ t structs
{
    controller_in.x_velo = (g_robot_state.input.vx);
    controller_in.y_velo = (g_robot_state.input.vy);
    controller_in.ang_velo = 0.0f; //

    updateWheelVelocity(&wheel_velo_desired, &controller_in);
    desaturate(&wheel_velo_desired);
    
    DJI_Motor_Set_Velocity(motor_w1, wheel_velo_desired.wheel_one_speed);
    DJI_Motor_Set_Velocity(motor_w2, wheel_velo_desired.wheel_two_speed);
    DJI_Motor_Set_Velocity(motor_w3, wheel_velo_desired.wheel_three_speed);
    DJI_Motor_Set_Velocity(motor_w4, wheel_velo_desired.wheel_four_speed);

}