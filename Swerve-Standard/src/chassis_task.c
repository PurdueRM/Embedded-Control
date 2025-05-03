#include "chassis_task.h"

#include "robot.h"
#include "remote.h"
#include "dji_motor.h"
#include "imu_task.h"
#include "motor.h"
#include "referee_system.h"
#include "swerve_locomotion.h"
#include "swerve_odometry.h"
#include "rate_limiter.h"

#include "FreeRTOS.h"
#include "task.h"

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;
extern Referee_System_t Referee_System;
extern IMU_t g_imu;

DJI_Motor_Handle_t *g_azimuth_motors[NUMBER_OF_MODULES];
DJI_Motor_Handle_t *g_drive_motors[NUMBER_OF_MODULES];
swerve_constants_t g_swerve_constants;
swerve_chassis_state_t g_chassis_state;
float measured_angles[NUMBER_OF_MODULES];

rate_limiter_t chassis_vel_limiters[4];
rate_limiter_t chassis_omega_limiter;

float chassis_rad = WHEEL_BASE * 1.414f; //TODO init?

float g_spintop_omega = SPIN_TOP_OMEGA;

float prev_yaw;
unsigned int last_odom_update;

void Chassis_Task_Init() {
    // init common PID configuration for azimuth motors
    Motor_Config_t azimuth_motor_config = {
        .control_mode = POSITION_VELOCITY_SERIES,
        .angle_pid =
            {
                .kp = 400.0f,
                .kd = 20.0f,
                .output_limit = 300.0f,
            },
        .velocity_pid =
            {
                .kp = 300.0f,
                .ki = 0.0f,
                .kd = 100.0f,
                .kf = 2000.0f,
                .feedforward_limit = 5000.0f,
                .integral_limit = 5000.0f,
                .output_limit = GM6020_MAX_CURRENT,
            }};

    // init common PID configuration for drive motors
    Motor_Config_t drive_motor_config = {
        .control_mode = VELOCITY_CONTROL,
        .velocity_pid = {
            .kp = 500.0f,
            .kd = 200.0f,
            .kf = 100.0f,
            .output_limit = M3508_MAX_CURRENT,
            .integral_limit = 3000.0f,
        }};

    // Initialize the swerve modules
    typedef struct
    {
        float azimuth_can_bus;
        float azimuth_speed_controller_id;
        float azimuth_offset;
        Motor_Reversal_t azimuth_motor_reversal;

        float drive_can_bus;
        float drive_speed_controller_id;
        Motor_Reversal_t drive_motor_reversal;
    } swerve_module_config_t;

    swerve_module_config_t module_configs[NUMBER_OF_MODULES] = {
        {2, 1, 2050, MOTOR_REVERSAL_REVERSED, 1, 1, MOTOR_REVERSAL_NORMAL},
        {2, 2, 1940, MOTOR_REVERSAL_REVERSED, 2, 2, MOTOR_REVERSAL_NORMAL},
        {2, 3, 1430, MOTOR_REVERSAL_REVERSED, 2, 3, MOTOR_REVERSAL_REVERSED},
        {2, 4, 8150, MOTOR_REVERSAL_REVERSED, 2, 4, MOTOR_REVERSAL_REVERSED}};

    // Initialize the swerve modules
    for (int i = 0; i < NUMBER_OF_MODULES; i++)
    {
        // configure azimuth motor
        azimuth_motor_config.can_bus = module_configs[i].azimuth_can_bus;
        azimuth_motor_config.offset = module_configs[i].azimuth_offset;
        azimuth_motor_config.speed_controller_id = module_configs[i].azimuth_speed_controller_id;
        azimuth_motor_config.motor_reversal = module_configs[i].azimuth_motor_reversal;
        g_azimuth_motors[i] = DJI_Motor_Init(&azimuth_motor_config, GM6020);

        // configure drive motor
        drive_motor_config.can_bus = module_configs[i].drive_can_bus;
        drive_motor_config.speed_controller_id = module_configs[i].drive_speed_controller_id;
        drive_motor_config.motor_reversal = module_configs[i].drive_motor_reversal;
        g_drive_motors[i] = DJI_Motor_Init(&drive_motor_config, M3508);
    }

    // // Initialize the swerve locomotion constants
    g_swerve_constants = swerve_init(TRACK_WIDTH, WHEEL_BASE, WHEEL_DIAMETER, SWERVE_MAX_SPEED, SWERVE_MAX_ANGLUAR_SPEED);

    // // Initialize the rate limiters
    for (int i = 0; i < NUMBER_OF_MODULES; i++)
    {
        rate_limiter_init(&chassis_vel_limiters[i], SWERVE_MAX_WHEEL_ACCEL);
    }
    #define SWERVE_MAX_OMEGA_ACCEL (5.0f)
    rate_limiter_init(&chassis_omega_limiter, SWERVE_MAX_OMEGA_ACCEL);

    g_robot_state.chassis.pose = (pose_2d){0, 0, 0};

    prev_yaw = g_imu.rad.yaw;
    last_odom_update = xTaskGetTickCount();
}

void Chassis_Ctrl_Loop()
{
    float vx = g_robot_state.input.vx;
    float vy = g_robot_state.input.vy;

    if (g_robot_state.IS_SUPER_CAPACITOR_ENABLED) {
        g_swerve_constants.max_speed = 5.0;
        for (int i = 0; i < NUMBER_OF_MODULES; i++) {
            chassis_vel_limiters[i].rate_limit = SWERVE_QUICK_STOP_ACCEL / 2.0f;
        }
    } // Quick Deceleration when the joystick is released
    else if ((vx * vx + vy * vy) < 0.01f) {
        for (int i = 0; i < NUMBER_OF_MODULES; i++) {
            chassis_vel_limiters[i].rate_limit = SWERVE_QUICK_STOP_ACCEL;
        }
    } else if (g_robot_state.chassis.IS_SPINTOP_ENABLED) {
        // g_swerve_constants.max_speed = MAX_SPEED_W45;
        for (int i = 0; i < NUMBER_OF_MODULES; i++) {
            chassis_vel_limiters[i].rate_limit = SWERVE_MAX_WHEEL_ACCEL*0.7f;
        }
    } else {
        // g_swerve_constants.max_speed = MAX_SPEED_W45;
        for (int i = 0; i < NUMBER_OF_MODULES; i++) {
            chassis_vel_limiters[i].rate_limit = SWERVE_MAX_WHEEL_ACCEL;
        }
    }

    // Control loop for the chassis
    for (int i = 0; i < NUMBER_OF_MODULES; i++) {
        measured_angles[i] = DJI_Motor_Get_Absolute_Angle(g_azimuth_motors[i]);
    }
    g_chassis_state.v_x = g_robot_state.chassis.x_speed * g_swerve_constants.max_speed;
    g_chassis_state.v_y = g_robot_state.chassis.y_speed * g_swerve_constants.max_speed;

    // Offset chassis orientation based on gimbal direction
    // Note: commented because currently handled in process remote input
    // float theta = g_robot_state.gimbal.yaw_angle;
    // g_chassis_state.v_x = g_chassis_state.v_x * cos(theta) - g_chassis_state.v_y * sin(theta);
    // g_chassis_state.v_y = g_chassis_state.v_x * sin(theta) + g_chassis_state.v_y * cos(theta);

    // If spintop enabled, chassis omega set to spintop value
    if (g_robot_state.chassis.IS_SPINTOP_ENABLED) {
        g_chassis_state.omega = rate_limiter_iterate(&chassis_omega_limiter, Rescale_Chassis_Velocity());
    } else {
        g_chassis_state.omega = rate_limiter_iterate(&chassis_omega_limiter, g_robot_state.chassis.omega * SWERVE_MAX_ANGLUAR_SPEED);
    }

    // Calculate the kinematics of the chassis
    swerve_calculate_kinematics(&g_chassis_state, &g_swerve_constants);
    swerve_optimize_module_angles(&g_chassis_state, measured_angles);
    //swerve_desaturate_wheel_speeds(&g_chassis_state, &g_swerve_constants);

    // Update odometry
    unsigned int curr_time = xTaskGetTickCount();
    float dt = (curr_time - last_odom_update) / (float)configTICK_RATE_HZ; // convert to seconds
    swerve_odometry_update(&g_robot_state.chassis.pose, g_chassis_state.states, &g_swerve_constants, g_robot_state.chassis.yaw, prev_yaw, dt);
    last_odom_update = curr_time;
    prev_yaw = g_imu.rad.yaw;
    
    // rate limit the module speeds
    for (int i = 0; i < NUMBER_OF_MODULES; i++) {
        g_chassis_state.states[i].speed = rate_limiter_iterate(&chassis_vel_limiters[i], g_chassis_state.states[i].speed);   
    }

    swerve_convert_to_rpm(&g_chassis_state, &g_swerve_constants);

    for (int i = 0; i < NUMBER_OF_MODULES; i++) {
        DJI_Motor_Set_Angle(g_azimuth_motors[i], g_chassis_state.states[i].angle);
        DJI_Motor_Set_Velocity(g_drive_motors[i], g_chassis_state.states[i].speed);
    }

    Update_Maxes();
}

void Update_Maxes()
{
    switch(Referee_System.Robot_State.Chassis_Power_Max) {
        case 45:
            g_swerve_constants.max_speed = MAX_SPEED_W45;
            g_spintop_omega = SPINTOP_OMEGA_W45;
            break;
        case 50:
            g_swerve_constants.max_speed = MAX_SPEED_W50;
            g_spintop_omega = SPINTOP_OMEGA_W50;
            break;
        case 55:
            g_swerve_constants.max_speed = MAX_SPEED_W55;
            g_spintop_omega = SPINTOP_OMEGA_W55;
            break;
        case 60:
            g_swerve_constants.max_speed = MAX_SPEED_W60;
            g_spintop_omega = SPINTOP_OMEGA_W60;
            break;
        case 65:
            g_swerve_constants.max_speed = MAX_SPEED_W65;
            g_spintop_omega = SPINTOP_OMEGA_W65;
            break;
        case 70:
            g_swerve_constants.max_speed = MAX_SPEED_W70;
            g_spintop_omega = SPINTOP_OMEGA_W70;
            break;
        case 75:
            g_swerve_constants.max_speed = MAX_SPEED_W75;
            g_spintop_omega = SPINTOP_OMEGA_W75;
            break;
        case 80:
            g_swerve_constants.max_speed = MAX_SPEED_W80;
            g_spintop_omega = SPINTOP_OMEGA_W80;
            break;
        case 90:
            g_swerve_constants.max_speed = MAX_SPEED_W90;
            g_spintop_omega = SPINTOP_OMEGA_W90;
            break;
        case 100:
            g_swerve_constants.max_speed = MAX_SPEED_W100;
            g_spintop_omega = SPINTOP_OMEGA_W100;
            break;
        default:
            g_swerve_constants.max_speed = MAX_SPEED_W45;
            g_spintop_omega = SPINTOP_OMEGA_W45;
    }
}

/*
 * scale spintop omega by inverse of translation speed to prioritize translation
 * spin_coeff = rw/(v + rw) // r = rad, w = desired omega (spin top omega), v = translational speed
 * chassis_omega *= spin_coeff
 */
float Rescale_Chassis_Velocity(void) {
    float translation_speed = sqrtf(powf(g_robot_state.chassis.x_speed, 2) + powf(g_robot_state.chassis.y_speed, 2));
    float spin_coeff = chassis_rad * g_spintop_omega / (translation_speed * 2.0f + chassis_rad * g_spintop_omega);
    float target_omega = g_spintop_omega * spin_coeff;
    return target_omega;
}