#include "robot.h"

#include "robot_tasks.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "launch_task.h"
#include "remote.h"
#include "gimbal_task.h"
#include "imu_task.h"
#include "referee_system.h"
#include "buzzer.h"
#include "supercap.h"
#include "dji_motor.h"
#include "dm_motor.h"

Robot_State_t g_robot_state = {0};
extern Remote_t g_remote;
extern Supercap_t g_supercap;

Input_State_t g_input_state = {0};

#define KEYBOARD_RAMP_COEF (0.01f)

/**
 * @brief This function initializes the robot.
 * This means setting the state to STARTING_UP,
 * initializing the buzzer, and calling the
 * Robot_Task_Start() for the task scheduling
 */
void Robot_Init()
{
    g_robot_state.state = STARTING_UP;

    Buzzer_Init();
    Melody_t system_init_melody = {
        .notes = SYSTEM_INITIALIZING,
        .loudness = 0.5f,
        .note_num = SYSTEM_INITIALIZING_NOTE_NUM,
    };
    Buzzer_Play_Melody(system_init_melody); // TODO: Change to non-blocking

    // Initialize all tasks
    Robot_Tasks_Start();
}

/**
 * @brief This function handles the starting up state of the robot, initializing all hardware.
 */
void Handle_Starting_Up_State()
{
    // Initialize all hardware
    Chassis_Task_Init();
    Gimbal_Task_Init();
    Launch_Task_Init();
    Remote_Init(&huart3);
    CAN_Service_Init();
    // Referee_System_Init(&huart1);
    // Supercap_Init(&g_supercap);
    // Jetson_Orin_Init(&huart1);
    Referee_System_Init(&huart6);
    // Set robot state to disabled
    g_robot_state.state = DISABLED;
}

/**
 * @brief This function handles the enabled state of the robot.
 * This means processing remote input, and subsystem control.
 */
void Handle_Enabled_State()
{
    if (g_remote.online_flag == REMOTE_OFFLINE || g_remote.controller.right_switch == DOWN)
    {
        g_robot_state.state = DISABLED;
    }
    else
    {
        // Process movement and components in enabled state
        Referee_Set_Robot_State();
        Process_Remote_Input();
        Process_Chassis_Control();
        Process_Gimbal_Control();
        Process_Launch_Control();
    }
}

/**
 * @brief This function handles the disabled state of the robot.
 * This means disabling all motors and components
 */
void Handle_Disabled_State()
{
    DJI_Motor_Disable_All();
    DM_Motor_Disable_All();

    Gimbal_Task_Disable();
    //  Disable all major components
    g_robot_state.launch.IS_FLYWHEEL_ENABLED = 0;
    g_robot_state.chassis.x_speed = 0;
    g_robot_state.chassis.y_speed = 0;

    if (g_remote.online_flag == REMOTE_ONLINE && g_remote.controller.right_switch != DOWN)
    {
        g_robot_state.state = ENABLED;
        DJI_Motor_Enable_All();
    }
}

void Process_Remote_Input()
{
    g_robot_state.input.vy_keyboard = ((1.0f - KEYBOARD_RAMP_COEF) * g_robot_state.input.vy_keyboard + g_remote.keyboard.W * KEYBOARD_RAMP_COEF - g_remote.keyboard.S * KEYBOARD_RAMP_COEF);
    g_robot_state.input.vx_keyboard = ((1.0f - KEYBOARD_RAMP_COEF) * g_robot_state.input.vx_keyboard - g_remote.keyboard.A * KEYBOARD_RAMP_COEF + g_remote.keyboard.D * KEYBOARD_RAMP_COEF);
    float temp_x = g_robot_state.input.vx_keyboard + g_remote.controller.left_stick.x/660.0f * 2.0f;
    float temp_y = g_robot_state.input.vy_keyboard + g_remote.controller.left_stick.y/660.0f * 2.0f;
    g_robot_state.input.vx = temp_x;
    g_robot_state.input.vy = temp_y;

    g_robot_state.input.vomega = -g_remote.controller.right_stick.x/660.0f * 6.28f + g_remote.mouse.x / 10000.0f;

    // TODO. Add supercapicitor keys/remote input here
    
    if (__IS_TRANSITIONED(g_remote.controller.left_switch, g_robot_state.input.prev_left_switch, MID))
    {
        g_robot_state.chassis.IS_SPINTOP_ENABLED = 1;
    }
    if (__IS_TRANSITIONED(g_remote.controller.left_switch, g_robot_state.input.prev_left_switch, DOWN) ||
        __IS_TRANSITIONED(g_remote.controller.left_switch, g_robot_state.input.prev_left_switch, UP))
    {
        g_robot_state.chassis.IS_SPINTOP_ENABLED = 0;
    }

    if ((g_remote.mouse.left) || (g_remote.controller.wheel > 50.0f)) { // Hold left mouse to fire
        g_robot_state.launch.fire_mode = SINGLE_FIRE;
    } else {
        g_robot_state.launch.fire_mode = NO_FIRE;
    }

    if (__IS_TOGGLED(g_remote.keyboard.B, g_input_state.prev_B))
    {
        g_robot_state.chassis.IS_SPINTOP_ENABLED ^= 0x01;
    }

    if (g_remote.controller.left_switch == UP) { // Left switch high to enable spintop
        //g_robot_state.chassis.IS_SPINTOP_ENABLED = 1;
        g_robot_state.launch.IS_FIRING_ENABLED = 1;
        g_robot_state.launch.IS_AUTO_AIMING_ENABLED = 1;
    } else {
        //g_robot_state.chassis.IS_SPINTOP_ENABLED = 0;
        g_robot_state.launch.IS_FIRING_ENABLED = 0;
        g_robot_state.launch.IS_AUTO_AIMING_ENABLED = 0;
    }

    g_robot_state.input.prev_left_switch = g_remote.controller.left_switch;
    g_robot_state.input.prev_B = g_remote.keyboard.B;

}

void Process_Chassis_Control()
{
     Chassis_Ctrl_Loop();
}

void Process_Gimbal_Control()
{
     Gimbal_Ctrl_Loop();
}

void Process_Launch_Control()
{
     Launch_Ctrl_Loop();
}

/**
 *  This function is called periodically by the Robot Task.
 *  It serves as the top level state machine for the robot based on the current state.
 *  Appropriate functions are called.
 */
void Robot_Command_Loop()
{
    switch (g_robot_state.state)
    {
    case STARTING_UP:
        Handle_Starting_Up_State();
        break;
    case DISABLED:
        Handle_Disabled_State();
        break;
    case ENABLED:
        Handle_Enabled_State();
        break;
    default:
        Error_Handler();
        break;
    }
}
