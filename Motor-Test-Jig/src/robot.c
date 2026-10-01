/* THIS ROBOT IS BASED ON THE SENTRY */

#include "robot.h"

#include "robot_tasks.h"
//#include "chassis_task.h"
//#include "gimbal_task.h"
//#include "launch_task.h"
//#include "remote.h"
#include "imu_task.h"
//#include "referee_system.h"
#include "buzzer.h"
//#include "supercap.h"
#include "dji_motor.h"
#include "dm_motor.h"

Robot_State_t g_robot_state = {0};
DJI_Motor_Handle_t *DJI_test_motor;
DM_Motor_Handle_t *DM_test_motor;
//extern Remote_t g_remote;
//extern Supercap_t g_supercap;

/*
 * @brief This file is dedicated to testing motors that we have in our possession.
 * At this time it is a very rudimentary test that makes the motor in
 * in question spin.
 *
 *
 * @howto First, make uncomment either the DJI_Motor_Test or DM_Motor_Test
 * macro depending on the type of motor you wish to test. Then, set the DJI
 * or DM _MOTOR_TYPE macro according to the specific model. The macros will
 * automatically update the maximum current and pid values for you. The final
 * thing that needs to be changed is either the speed controller id for DJI
 * motors or the tx and rx buffers to match whatever the controller 
 * is configured to. You still have to do that manually.  :(
 */

// This needs to be uncommented to test DJI motors
#define DJI_Motor_Test

// Macros defining the pid values by motor type
#define M3508_KP 500.0f
#define M3508_KI 0.0f
#define M3508_KD 200.0f
#define M3508_KF 100.0f

#define M2006_KP 500.0f
#define M2006_KI 0.0f
#define M2006_KD 200.0f
#define M2006_KF 100.0f

#define GM6020_KP 80.0f
#define GM6020_KI 2.0f
#define GM6020_KD 1.0f
#define GM6020_KF 0.0f

// Modify the following macro to match what type of motor you are testing
#define DJI_MOTOR_TYPE GM6020

// Modify the following macro to match the speed controller id
#define SPEED_CONTROLLER_ID 1

// Setup macro
#define CONCAT(a, b) a##b

// Macros for setting the max current
#define MOTOR_MAX_CURRENT(type) CONCAT(type, _MAX_CURRENT_INT)
#define DJI_MOTOR_MAX_CURRENT MOTOR_MAX_CURRENT(DJI_MOTOR_TYPE)

// Macros for setting
#define MOTOR_KP(type) CONCAT(type, _KP)
#define DJI_MOTOR_KP MOTOR_KP(DJI_MOTOR_TYPE)

#define MOTOR_KI(type) CONCAT(type, _KI)
#define DJI_MOTOR_KI MOTOR_KI(DJI_MOTOR_TYPE)

#define MOTOR_KD(type) CONCAT(type, _KD)
#define DJI_MOTOR_KD MOTOR_KD(DJI_MOTOR_TYPE)

#define MOTOR_KF(type) CONCAT(type, _KF)
#define DJI_MOTOR_KF MOTOR_KF(DJI_MOTOR_TYPE)

// This needs to be uncommented to test DM motors
// #define DM_Motor_Test

// The following determines what can bus you are using
#define CAN_BUS 1

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
        .loudness = 0.0f,
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
    /* START NEW */
    Motor_Task_Init();
    /* END NEW */
    
    //Gimbal_Task_Init();
    //Launch_Task_Init();
    //Remote_Init(&huart3);
    CAN_Service_Init();
    // Referee_System_Init(&huart1);
    // Supercap_Init(&g_supercap);
    //Jetson_Orin_Init(&huart1);
    //Referee_System_Init(&huart6);
    //Supercap_Init(&huart1);

    // Set robot state to disabled
    g_robot_state.state = ENABLED;
}

/**
 * @brief This function handles the enabled state of the robot.
 * This means processing remote input, and subsystem control.
 */
void Handle_Enabled_State()
{
    /* START NEW */
    Process_Motor_Control();
    /* END NEW */
    
    //Referee_Set_Robot_State();
    //Process_Remote_Input();
    //Process_Chassis_Control();
    //Process_Gimbal_Control();
    //Process_Launch_Control();
}

#define M3508_TEST_KP 500.0f
#define M3508_TEST_KI 0.0f
#define M3508_TEST_KD 200.0f
#define M3508_TEST_KF 100.0f

#define M2006_TEST_KP 500.0f
#define M2006_TEST_KI 0.0f
#define M2006_TEST_KD 200.0f
#define M2006_TEST_KF 100.0f

#define GM6020_TEST_KP 80.0f
#define GM6020_TEST_KI 2.0f
#define GM6020_TEST_KD 1.0f
#define GM6020_TEST_KF 0.0f

void Motor_Task_Init()
{
    #ifdef DJI_Motor_Test
    Motor_Config_t DJI_test_motor_config = {
        .can_bus = CAN_BUS,
        .speed_controller_id = SPEED_CONTROLLER_ID,
        .offset = 0,
        .control_mode = VELOCITY_CONTROL, /*POSITION_CONTROL_TOTAL_ANGLE,*/
        .motor_reversal = MOTOR_REVERSAL_NORMAL,
        .velocity_pid =
            {
                .kp = DJI_MOTOR_KP, // M3508=500, M2006=500, GM6020=80
                .ki = DJI_MOTOR_KI, // M3508=0, M2006=0, GM6020=2
                .kd = DJI_MOTOR_KD, // M3508=200, M2006=200, GM6020=1
                .kf = DJI_MOTOR_KF, // M3508=100, M2006=100, GM6020=0
                .feedforward_limit = 10000.0f,
                .integral_limit = 1000.0f,
                .output_limit = DJI_MOTOR_MAX_CURRENT,
            },
    };

    DJI_test_motor = DJI_Motor_Init(&DJI_test_motor_config, DJI_MOTOR_TYPE);
    DJI_Motor_Enable_All();
    #endif

    #ifdef DM_Motor_Test
    DM_Motor_Config_t DM_test_motor_config = {
        .can_bus = CAN_BUS,
        .control_mode = DM_MOTOR_MIT,
        .rx_id = 0x51,
        .tx_id = 0x01,
        .disable_behavior = DM_MOTOR_ZERO_CURRENT,
        .kp = 10.0f,
        .kd = 1.0f,
    };
    DM_test_motor = DM_Motor_Init(&DM_test_motor_config);
    DM_Motor_Enable_Motor(&DM_test_motor);
    #endif
}

void Process_Motor_Control() 
{   
    #ifdef DJI_Motor_Test
    DJI_Motor_Set_Velocity(DJI_test_motor, 30.0f);
    #endif

    #ifdef DM_Motor_Test
    // If velocity does not work, try position or torqueb
    DM_Motor_Ctrl_MIT_PD(DM_testMotor, 0.0f, 5.0f, 0.0f, 20.0f, 8.5f);
    #endif
}


/**
 * @brief This function handles the ENABLED state of the robot.
 * This means disabling all motors and components
 */
 /*
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
*/

/*
void Process_Remote_Input()
{
    g_robot_state.input.vx = g_remote.controller.left_stick.x/660.0f * 2.0f;
    g_robot_state.input.vy = g_remote.controller.left_stick.y/660.0f * 2.0f;
    g_robot_state.input.vomega = -g_remote.controller.right_stick.x/660.0f * 6.28f;
    
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

}
*/

/*
void Process_Chassis_Control()
{
     Chassis_Ctrl_Loop();
}
*/

/*
void Process_Gimbal_Control()
{
     Gimbal_Ctrl_Loop();
}
*/

/*
void Process_Launch_Control()
{
     Launch_Ctrl_Loop();
}
*/

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
    case ENABLED:
        Handle_Enabled_State();
        break;
    default:
        Error_Handler();
        break;
    }
}
