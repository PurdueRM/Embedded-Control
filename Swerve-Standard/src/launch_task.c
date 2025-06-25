#include "launch_task.h"

#include "dji_motor.h"
#include "motor.h"
#include "robot.h"
#include "remote.h"
#include "user_math.h"
#include "referee_system.h"
#include "laser.h"
#include <stdint.h>

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;
Launch_Target_t g_launch_target;
DJI_Motor_Handle_t *g_flywheel_left, *g_flywheel_right, *g_feed_motor;

void Launch_Task_Init()
{
    // Init Launch Hardware
    Motor_Config_t flywheel_left_config = {
        .can_bus = 1,
        .speed_controller_id = 4,
        .offset = 0,
        .control_mode = VELOCITY_CONTROL,
        .motor_reversal = MOTOR_REVERSAL_NORMAL,
        .velocity_pid =
            {
                .kp = 500.0f,
                .output_limit = M3508_MAX_CURRENT_INT,
            },
    };

    Motor_Config_t flywheel_right_config = {
        .can_bus = 1,
        .speed_controller_id = 5,
        .offset = 0,
        .control_mode = VELOCITY_CONTROL,
        .motor_reversal = MOTOR_REVERSAL_REVERSED,
        .velocity_pid =
            {
                .kp = 500.0f,
                .output_limit = M3508_MAX_CURRENT_INT,
            },
    };

        Motor_Config_t feed_speed_config = {
        .can_bus = 1,
        .speed_controller_id = 2,
        .offset = 0,
        .control_mode = VELOCITY_CONTROL | POSITION_CONTROL,
        .motor_reversal = MOTOR_REVERSAL_NORMAL,
        .velocity_pid =
            {
                .kp = 5000.0f,
                .kd = 20.0f,
                .kf = 100.0f,
                .output_limit = M2006_MAX_CURRENT_INT,
            },
        .angle_pid =
            {
                .kp = 450000.0f,
                .kd = 5000000.0f,
                .ki = 0.1f,
                .kf = 1000.0f,
                .output_limit = M2006_MAX_CURRENT_INT,
                .integral_limit = 1000.0f,
            }
    };

    g_flywheel_left = DJI_Motor_Init(&flywheel_left_config,M3508);
    g_flywheel_right = DJI_Motor_Init(&flywheel_right_config,M3508);
    g_feed_motor = DJI_Motor_Init(&feed_speed_config,M2006);

    Laser_Init();
}

void Feed_Angle_Calc()
{
    // Update Counter
    if (g_remote.controller.wheel > 50.0f || g_remote.mouse.left) {
        g_launch_target.burst_launch_flag = 1;
    } else {
        g_launch_target.burst_launch_flag = 0;
    }
    
    g_launch_target.heat_count++;
    g_launch_target.launch_freq_count++;
    if (Referee_System.Online_Flag)
    {
        if (Referee_System.Robot_State.Shooter_Power_Output == 0 || !g_launch_target.burst_launch_flag)
        {
            g_launch_target.feed_angle = g_feed_motor->stats->total_angle_rad;
        } 
        if (g_launch_target.heat_count*2 % 100 == 0)
        {
            g_launch_target.calculated_heat -= Referee_Robot_State.Cooling_Rate/10;
            __MAX_LIMIT(g_launch_target.calculated_heat,0,Referee_Robot_State.Heat_Max);
        }
        if (g_launch_target.burst_launch_flag && !g_launch_target.reverse_flag) 
        {
            if (g_launch_target.launch_freq_count*2 > LAUNCH_PERIOD)
            {
                g_launch_target.launch_freq_count = 0;
                if((Referee_Robot_State.Heat_Max-g_launch_target.calculated_heat) > 20)
                {
                    g_launch_target.calculated_heat += 10;
                    g_launch_target.feed_angle += FEED_1_PROJECTILE_ANGLE;
                }
            }
            DJI_Motor_Set_Control_Mode(g_feed_motor, POSITION_CONTROL_TOTAL_ANGLE);
            DJI_Motor_Set_Angle(g_feed_motor,g_launch_target.feed_angle);
        }
        if(g_launch_target.reverse_flag && !g_launch_target.prev_reverse_flag)
        // if (g_launch_target.reverse_burst_launch_pending_flag)
        {
            // g_launch_target.reverse_burst_launch_pending_flag = 0;
            g_launch_target.feed_angle -= FEED_1_PROJECTILE_ANGLE;
            DJI_Motor_Set_Control_Mode(g_feed_motor, POSITION_CONTROL_TOTAL_ANGLE);
            DJI_Motor_Set_Angle(g_feed_motor,g_launch_target.feed_angle);
        }
    }
    g_launch_target.prev_burst_launch_flag = g_launch_target.burst_launch_flag;
    g_launch_target.prev_reverse_flag = g_launch_target.reverse_flag;
}

void Launch_Ctrl_Loop()
{
    if (g_remote.controller.left_switch == UP) {
        DJI_Motor_Set_Velocity(g_flywheel_left, -100);
        DJI_Motor_Set_Velocity(g_flywheel_right, -100);
    } else {
        DJI_Motor_Set_Velocity(g_flywheel_left, 0);
        DJI_Motor_Set_Velocity(g_flywheel_right, 0);
    }

    DJI_Motor_Set_Velocity(g_feed_motor, g_remote.controller.wheel/660.0f * 100.0f);
    Feed_Angle_Calc();
    
}