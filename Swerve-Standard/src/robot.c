#include "robot.h"

#include "robot_tasks.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "launch_task.h"
#include "gimbal_task.h"
#include "referee_system.h"
#include "remote.h"
#include "buzzer.h"
#include "supercap.h"
#include "user_math.h"
#include "math.h"
#include "rate_limiter.h"

Robot_State_t g_robot_state = {0};
extern Remote_t g_remote;
extern Supercap_t g_supercap;

extern DJI_Motor_Handle_t *g_yaw;

Input_State_t g_input_state = {0};
rate_limiter_t controller_limit_x = {0};
rate_limiter_t controller_limit_y = {0};

#define KEYBOARD_RAMP_COEF (0.004f)

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
    CAN_Service_Init();
    //Referee_System_Init(&huart1);
    Supercap_Init(&g_supercap);
    Chassis_Task_Init();
    Gimbal_Task_Init();
    Launch_Task_Init();
    Jetson_Orin_Init(&huart1); // ! temp uart1

    Remote_Init(&huart3);

    // ! this should be 4, we just made it high to disable the rate limiter for now
    #define MAX_ACCEL 100 // %/s^2 defined here for local context
    rate_limiter_init(&g_robot_state.rate_limiters.controller_limit_x, MAX_ACCEL);
    rate_limiter_init(&g_robot_state.rate_limiters.controller_limit_y, MAX_ACCEL);

    g_robot_state.state = DISABLED;
}

/**
 * @brief This function handles the enabled state of the robot.
 * This means processing remote input, and subsystem control.
 */
void Handle_Enabled_State()
{
    if ((g_remote.online_flag == REMOTE_OFFLINE) || (g_remote.controller.right_switch == DOWN))
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

    //  Disable all major components
    g_robot_state.launch.IS_FLYWHEEL_ENABLED = 0;
    g_robot_state.chassis.x_speed = 0;
    g_robot_state.chassis.y_speed = 0;

    if ((g_remote.online_flag == REMOTE_ONLINE) && (g_remote.controller.right_switch != DOWN))
    {
        g_robot_state.state = ENABLED;
        DJI_Motor_Enable_All();
    }
}

void Process_Remote_Input()
{
    // Process remote input
    g_robot_state.input.vy_keyboard = ((1.0f - KEYBOARD_RAMP_COEF) * g_robot_state.input.vy_keyboard + g_remote.keyboard.W * KEYBOARD_RAMP_COEF - g_remote.keyboard.S * KEYBOARD_RAMP_COEF);
    g_robot_state.input.vx_keyboard = ((1.0f - KEYBOARD_RAMP_COEF) * g_robot_state.input.vx_keyboard - g_remote.keyboard.A * KEYBOARD_RAMP_COEF + g_remote.keyboard.D * KEYBOARD_RAMP_COEF);
    float temp_x = g_robot_state.input.vx_keyboard + g_remote.controller.left_stick.x / REMOTE_STICK_MAX;
    float temp_y = g_robot_state.input.vy_keyboard + g_remote.controller.left_stick.y / REMOTE_STICK_MAX;
    g_robot_state.input.vx = rate_limiter(&g_robot_state.rate_limiters.controller_limit_x, temp_x);
    g_robot_state.input.vy = rate_limiter(&g_robot_state.rate_limiters.controller_limit_y, temp_y);


    // Calculate Gimbal Oriented Control
    float theta = DJI_Motor_Get_Absolute_Angle(g_yaw);
    g_robot_state.chassis.x_speed = -g_robot_state.input.vy * sin(theta) + g_robot_state.input.vx * cos(theta);
    g_robot_state.chassis.y_speed = g_robot_state.input.vy * cos(theta) + g_robot_state.input.vx * sin(theta);

    g_robot_state.gimbal.yaw_angle -= (g_remote.controller.right_stick.x / 50000.0f + g_remote.mouse.x / 10000.0f);    // controller and mouse
    g_robot_state.gimbal.pitch_angle -= (g_remote.controller.right_stick.y / 100000.0f - g_remote.mouse.y / 50000.0f);

    // keyboard toggles
    if (__IS_TOGGLED(g_remote.keyboard.B, g_input_state.prev_B))
    {
        g_robot_state.launch.IS_FIRING_ENABLED ^= 0x01; // Toggle firing
    }
    if (__IS_TOGGLED(g_remote.keyboard.B, g_input_state.prev_B))
    {
        g_robot_state.chassis.IS_SPINTOP_ENABLED ^= 0x01; // Toggle spintop
    }
    if (__IS_TOGGLED(g_remote.keyboard.B, g_input_state.prev_B))
    {
        g_robot_state.UI_ENABLED ^= 0x01; // Toggle UI
    }
    if (__IS_TOGGLED(g_remote.keyboard.Shift, g_input_state.prev_Shift))
    {
        g_robot_state.IS_SUPER_CAPACITOR_ENABLED ^= 0x01; // Toggle supercap
    }

    // controller toggles
    if (__IS_TRANSITIONED(g_remote.controller.left_switch, g_input_state.prev_left_switch, MID))
    {
        g_robot_state.chassis.IS_SPINTOP_ENABLED = 1;
    }
    if (__IS_TRANSITIONED(g_remote.controller.left_switch, g_input_state.prev_left_switch, DOWN) ||
        __IS_TRANSITIONED(g_remote.controller.left_switch, g_input_state.prev_left_switch, UP))
    {
        g_robot_state.chassis.IS_SPINTOP_ENABLED = 0;
    }

    if (g_remote.controller.left_switch == UP)
    {
        g_robot_state.launch.IS_FIRING_ENABLED = 1;
    }
    else
    {
        g_robot_state.launch.IS_FIRING_ENABLED = 0;
    }

    if ((g_remote.controller.right_switch == UP) || (g_remote.mouse.right == 1)) // mouse right button auto aim
    {
        g_robot_state.launch.IS_AUTO_AIMING_ENABLED = 1;
    }
    else
    {
        g_robot_state.launch.IS_AUTO_AIMING_ENABLED = 0;
    }

    if (g_remote.controller.wheel < -50.0f)
    { // dial wheel forward single fire
        g_robot_state.launch.fire_mode = SINGLE_FIRE;
    }
    else if (g_remote.controller.wheel > 50.0f)
    { // dial wheel backward burst `fire
        g_robot_state.launch.fire_mode = FULL_AUTO;
    }
    else
    { // dial wheel mid stop fire
        g_robot_state.launch.fire_mode = NO_FIRE;
    }

    // cycle burst flags with keyboard
    // TODO: assign a key for this
    // if (__IS_TOGGLED(g_remote.keyboard.G, g_input_state.prev_G))
    // {
    //     if (g_robot_state.launch.fire_mode == FULL_AUTO)
    //     {
    //         g_robot_state.launch.fire_mode = SINGLE_FIRE;
    //     }
    //     else
    //     {
    //         g_robot_state.launch.fire_mode++;
    //     }
    // }



    // TODO: implement controller toggle for supercap
    // if (g_remote.controller.wheel > 50.0f && !g_robot_state.launch.IS_FLYWHEEL_ENABLED)
    // {
    //     g_supercap.supercap_enabled_flag = 1;
    // }
    // else
    // {
    //     g_supercap.supercap_enabled_flag = 0;
    // }

    // Update previous states keyboard
    g_input_state.prev_B = g_remote.keyboard.B;
    g_input_state.prev_G = g_remote.keyboard.G;
    g_input_state.prev_V = g_remote.keyboard.V;
    g_input_state.prev_Z = g_remote.keyboard.Z;
    g_input_state.prev_Shift = g_remote.keyboard.Shift;

    // Update previous states remote
    g_input_state.prev_left_switch = g_remote.controller.left_switch;
    g_input_state.prev_right_switch = g_remote.controller.right_switch;
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

/*
 * @brief It serves as the top level state machine for the robot based on the current state.
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
