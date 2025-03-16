#include "launch_task.h"

#include "dji_motor.h"
#include "robot.h"
#include "remote.h"
#include "user_math.h"
#include "referee_system.h"
#include "laser.h"
#include <stdint.h>

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;

DJI_Motor_Handle_t *g_flywheel_left, *g_flywheel_right, *g_feed_motor;

void Launch_Task_Init()
{
    // Init Launch Hardware
    Motor_Config_t flywheel_left_config = {
        .can_bus = 1,
        .speed_controller_id = 4,
        .offset = 0,
        .control_mode = VELOCITY_CONTROL,
        .motor_reversal = MOTOR_REVERSAL_REVERSED,
        .velocity_pid =
            {
                .kp = 500.0f,
                .output_limit = M3508_MAX_CURRENT,
            },
    };

    Motor_Config_t flywheel_right_config = {
        .can_bus = 1,
        .speed_controller_id = 5,
        .offset = 0,
        .control_mode = VELOCITY_CONTROL,
        .motor_reversal = MOTOR_REVERSAL_NORMAL,
        .velocity_pid =
            {
                .kp = 500.0f,
                .output_limit = M3508_MAX_CURRENT,
            },
    };

    Motor_Config_t feed_speed_config = {
        .can_bus = 1,
        .speed_controller_id = 2,
        .offset = 0,
        .control_mode = VELOCITY_CONTROL | POSITION_CONTROL_TOTAL_ANGLE,
        .motor_reversal = MOTOR_REVERSAL_NORMAL,
        .velocity_pid =
            {
                .kp = 500.0f,
                .kd = 200.0f,
                .kf = 100.0f,
                .output_limit = M2006_MAX_CURRENT,
            },
        .angle_pid =
            {
                .kp = 450000.0f,
                .kd = 15000000.0f,
                .ki = 0.1f,
                .output_limit = M2006_MAX_CURRENT,
                .integral_limit = 1000.0f,
            }
    };

    g_flywheel_left = DJI_Motor_Init(&flywheel_left_config,M3508);
    g_flywheel_right = DJI_Motor_Init(&flywheel_right_config,M3508);
    g_feed_motor = DJI_Motor_Init(&feed_speed_config,M2006);

    Laser_Init();
}

void Launch_Ctrl_Loop()
{
    if (!g_robot_state.launch.IS_FIRING_ENABLED)
    {
        stopFlywheel();
        Laser_Off();
        g_robot_state.launch.IS_FLYWHEEL_ENABLED = 0;
        return;
    } else {
        g_robot_state.launch.IS_FLYWHEEL_ENABLED = 1;
        startFlywheel();
        Laser_On();
    }

    if (g_robot_state.launch.IS_BUSY) { // check if we are in middle of a fire mode
        switch (g_robot_state.launch.busy_mode)
        {
        case REJIGGLE:
            rejiggle();
            break;
        case SINGLE_FIRE:
            handleSingleFire();
            break;
        case BURST_FIRE:
            break;
        case FULL_AUTO:
            handleFullAuto();
            break;
        default:
            break;
        }
    } else {
        // Control loop for launch to see if new mode is set
        switch (g_robot_state.launch.fire_mode)
        {
        case SINGLE_FIRE:
            handleSingleFire();
            break;
        case BURST_FIRE:
            // TODO: Complete 5 burst
            break;
        case FULL_AUTO:
            handleFullAuto();
            break;
        default:
            break;

        }
    }
}

void resetRelPos() {
    g_robot_state.launch.shooter_state.accum_angle = 0;
    g_robot_state.launch.shooter_state.prev_time = xTaskGetTickCount();
}

#define ticksToDegrees(ticks) ((ticks) * 360.0f / DJI_MAX_TICKS)
#define ticksToRad(ticks) ((ticks) * 360.0f / DJI_MAX_TICKS)
#define degreesToTicks(degrees) ((degrees) * DJI_MAX_TICKS / 360.0f)
float g_curr_angle = 0;
// TODO check if at ref
void handleSingleFire() {
    if (g_robot_state.launch.IS_BUSY) {
        if (DJI_Motor_Is_At_Angle(g_feed_motor, FEED_TOLERANCE)) // if shots fired :O then rejiggle
        {
            g_robot_state.launch.IS_BUSY = 0;
            g_robot_state.launch.busy_mode = IDLE;
            rejiggle();
        }
    }
    else {
        g_robot_state.launch.IS_BUSY = 1;
        g_robot_state.launch.busy_mode = SINGLE_FIRE;
        // set a new position reference x degrees forward
        resetRelPos();

        DJI_Motor_Set_Control_Mode(g_feed_motor, POSITION_CONTROL_TOTAL_ANGLE);
        g_curr_angle = DJI_Motor_Get_Total_Angle(g_feed_motor); // rad
        DJI_Motor_Set_Angle(g_feed_motor, g_curr_angle + SHOT_ANGLE_OFFSET_RAD);
        // DJI_Motor_Set_Velocity(g_feed_motor, FEED_RATE);
    }
}

void rejiggle() {
    //set a position reference slightly back to prevent jams
    float curr_angle_rad = DJI_Motor_Get_Total_Angle(g_feed_motor); // rad

    if (g_robot_state.launch.IS_BUSY) { // if busy, means we already called so go back
        if (DJI_Motor_Is_At_Angle(g_feed_motor, FEED_TOLERANCE))
        {
            g_robot_state.launch.busy_mode = IDLE;
            g_robot_state.launch.IS_BUSY = 0;
            DJI_Motor_Set_Control_Mode(g_feed_motor, POSITION_CONTROL_TOTAL_ANGLE);
            DJI_Motor_Set_Angle(g_feed_motor, curr_angle_rad + SHOT_ANGLE_OFFSET_RAD);
        }
    }
    else {
        g_robot_state.launch.busy_mode = REJIGGLE;
        g_robot_state.launch.IS_BUSY = 1;
        DJI_Motor_Set_Control_Mode(g_feed_motor, POSITION_CONTROL_TOTAL_ANGLE);
        DJI_Motor_Set_Angle(g_feed_motor, curr_angle_rad - SHOT_ANGLE_OFFSET_RAD);
    }
}

void handleFullAuto() {
    if (g_robot_state.launch.IS_BUSY) {
        if (g_robot_state.launch.fire_mode == NO_FIRE) {
            DJI_Motor_Set_Control_Mode(g_feed_motor, VELOCITY_CONTROL);
            DJI_Motor_Set_Velocity(g_feed_motor, 0);
            g_robot_state.launch.IS_BUSY = 0;
            g_robot_state.launch.busy_mode = IDLE;
            rejiggle();
        }
    } else {
        DJI_Motor_Set_Control_Mode(g_feed_motor, VELOCITY_CONTROL);
        DJI_Motor_Set_Velocity(g_feed_motor, FEED_RATE);
        g_robot_state.launch.IS_BUSY = 1;
        g_robot_state.launch.busy_mode = FULL_AUTO;
    }
}

void startFlywheel() {
    DJI_Motor_Set_Velocity(g_flywheel_left, -100);
    DJI_Motor_Set_Velocity(g_flywheel_right, -100);
}

void stopFlywheel() {
    DJI_Motor_Set_Velocity(g_flywheel_left, 0);
    DJI_Motor_Set_Velocity(g_flywheel_right, 0);
}
