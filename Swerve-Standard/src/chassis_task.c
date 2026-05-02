#include "chassis_task.h"

#include "robot.h"
#include "remote.h"
#include "dji_motor.h"
#include "motor.h"
#include "referee_system.h"
#include "supercap.h"
#include "swerve_locomotion.h"
#include "rate_limiter.h"
#include "pid.h"
#include "pid.h"

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;
extern Referee_System_t Referee_System;
extern DJI_Motor_Handle_t *g_yaw;
extern DJI_Motor_Handle_t *g_yaw;
uint16_t delay_counter = 0;


DJI_Motor_Handle_t *g_azimuth_motors[NUMBER_OF_MODULES];
DJI_Motor_Handle_t *g_drive_motors[NUMBER_OF_MODULES];
swerve_constants_t g_swerve_constants;
swerve_chassis_state_t g_chassis_state;
float measured_angles[NUMBER_OF_MODULES];

// static float filtered_error = 0.0f; // Persistant filtered error
// static float last_snap_angle = 0.0f; // Saved prev angle for Hysteresis

float gimbal_angle_difference;

PID_t angle_locking_pid;

rate_limiter_t chassis_vel_limiters[4];
rate_limiter_t chassis_omega_limiter;

float chassis_rad = WHEEL_BASE * 1.414f; //TODO init?

float g_spintop_omega = SPIN_TOP_OMEGA;
float g_supercap_linear_boost_rate = 1.0f;
float g_supercap_spintop_boost_rate = 1.0f;

void Chassis_Task_Init()
{
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
                .kp = 100.0f,
                .ki = 0.0f,
                .kd = 0.0f,
                .kf = 0.0f,
                .feedforward_limit = 5000.0f,
                .integral_limit = 5000.0f,
                .output_limit = GM6020_MAX_VOLTAGE_INT,
            }};

    // init common PID configuration for drive motors
    Motor_Config_t drive_motor_config = {
        .control_mode = VELOCITY_CONTROL,
        .velocity_pid = {
            .kp = 500.0f,
            .kd = 200.0f,
            .kf = 100.0f,
            .output_limit = M3508_MAX_CURRENT_INT,
            .integral_limit = 0.0f,
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
        {2, 2, 3991, MOTOR_REVERSAL_REVERSED, 2, 2, MOTOR_REVERSAL_NORMAL},
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

    // Initialize the swerve locomotion constants
    g_swerve_constants = swerve_init(TRACK_WIDTH, WHEEL_BASE, WHEEL_DIAMETER, SWERVE_MAX_SPEED, SWERVE_MAX_ANGLUAR_SPEED);

    // Initialize the rate limiters
    for (int i = 0; i < NUMBER_OF_MODULES; i++)
    {
        rate_limiter_init(&chassis_vel_limiters[i], SWERVE_MAX_WHEEL_ACCEL);
    }
    #define SWERVE_MAX_OMEGA_ACCEL (5.0f)
    rate_limiter_init(&chassis_omega_limiter, SWERVE_MAX_OMEGA_ACCEL);

    // Initialize PID for locking
    PID_Init(&angle_locking_pid, 5, 0, 300, 2 * PI * 1.5, 0, 0.01);

    g_robot_state.chassis.locked_state = DEFAULT_CHASSIS_MODE;
}

void Chassis_Ctrl_Loop()
{
    // float vx = g_robot_state.input.vx;
    // float vy = g_robot_state.input.vy;
    if (delay_counter > 0) {
        delay_counter--;
    }
    if ((Referee_System.Robot_State.Chassis_Power_Output == 0) || delay_counter > 0)
    {
        if (Referee_System.Robot_State.Chassis_Power_Output == 0)
        {
            float delay_time = 0.5f; //seconds
            delay_counter = delay_time * 500;
        }
        // disable turning motor
        for (int i = 0; i < NUMBER_OF_MODULES; i++)
        {
            g_chassis_state.omega = 0;
            g_chassis_state.v_x = 0;
            g_chassis_state.v_y = 0;
            g_robot_state.chassis.IS_SPINTOP_ENABLED = 0;
            DJI_Motor_Disable(g_drive_motors[i]);
            DJI_Motor_Disable(g_azimuth_motors[i]);
            PID_Reset(g_azimuth_motors[i]->angle_pid);
            PID_Reset(g_azimuth_motors[i]->velocity_pid);
            PID_Reset(g_drive_motors[i]->velocity_pid);
        }
        return;
    }
    if (g_robot_state.IS_SUPER_CAPACITOR_ENABLED) {
        g_supercap_linear_boost_rate = g_supercap_linear_boost_rate * 0.95f + 3.0f * 0.05f;
        g_supercap_spintop_boost_rate = g_supercap_spintop_boost_rate * 0.95f + 3.0f * 0.05f;
    }
    else {
        g_supercap_linear_boost_rate = g_supercap_linear_boost_rate * 0.95f + 1.0f * 0.05f;
        g_supercap_spintop_boost_rate = g_supercap_spintop_boost_rate * 0.95f + 1.0f * 0.05f;
    }

    // Control loop for the chassis
    for (int i = 0; i < NUMBER_OF_MODULES; i++) {
        measured_angles[i] = DJI_Motor_Get_Absolute_Angle(g_azimuth_motors[i]);
    }
    g_chassis_state.v_x = g_robot_state.chassis.x_speed * g_swerve_constants.max_speed * g_supercap_linear_boost_rate;
    g_chassis_state.v_y = g_robot_state.chassis.y_speed * g_swerve_constants.max_speed * g_supercap_linear_boost_rate;

    // Handle locking logic
    // TODO add an adjustable offset with keyboard
    // float lock_increment = PI / 2;
    // if (g_robot_state.chassis.IS_SPINTOP_ENABLED) {
    //     g_chassis_state.omega = rate_limiter_iterate(&chassis_omega_limiter, Rescale_Chassis_Velocity());
    // } else {
    //     g_chassis_state.omega = rate_limiter_iterate(&chassis_omega_limiter, 0);
    // }
    g_chassis_state.omega = rate_limiter_iterate(&chassis_omega_limiter, 0);    //For engineer, don't need swerve to spintop

    // Calculate the kinematics of the chassis
    swerve_calculate_kinematics(&g_chassis_state, &g_swerve_constants);

    // Optimize angles and desaturate wheel speeds at low speeds
    // Also optimizes angles during spintop
    // if (((get_fastest_wheel_speed() < 1.5f) && (fabs(vx) < 0.5) && (fabs(vy) < .5)) || g_robot_state.chassis.IS_SPINTOP_ENABLED) {
    //     swerve_optimize_module_angles(&g_chassis_state, measured_angles);
    // }
    
    // // rate limit the module speeds
    // for (int i = 0; i < NUMBER_OF_MODULES; i++) {
    //     g_chassis_state.states[i].speed = rate_limiter_iterate(&chassis_vel_limiters[i], g_chassis_state.states[i].speed);   
    // }

    swerve_convert_to_rpm(&g_chassis_state, &g_swerve_constants);

    for (int i = 0; g_remote.controller.left_switch == 2 && i < NUMBER_OF_MODULES; i++) {
        DJI_Motor_Set_Angle(g_azimuth_motors[i], g_chassis_state.states[i].angle);
        DJI_Motor_Set_Velocity(g_drive_motors[i], g_chassis_state.states[i].speed);
    }

    Update_Maxes();
}

/**
 * @brief Locks the chassis at lock_angle increments with a given offset
 * @param lock_angle: The angle and it's increments which the robot will lock to
 * @param offset: The chassis offset angle
 */
void Lock_Chassis_To_Angle(float lock_angle, float offset_angle)
{
    static float prev_error = 0.0f;
    static float last_snap_angle = 0.0f;

    // Get the angle difference and apply the offset
    gimbal_angle_difference = DJI_Motor_Get_Absolute_Angle(g_yaw) + offset_angle;
    __MAP_ANGLE_TO_UNIT_CIRCLE(gimbal_angle_difference);

    // 2. Check if angle difference exceeds hysteresis threshold
    float diff_from_last = (gimbal_angle_difference - last_snap_angle);
    __MAP_ANGLE_TO_UNIT_CIRCLE(diff_from_last);
    if (fabsf(diff_from_last) > (lock_angle / 2.0f) + HYSTERESIS_RAD) {
        last_snap_angle = roundf(gimbal_angle_difference / lock_angle) * lock_angle;
    }

    float error_angle = gimbal_angle_difference - last_snap_angle ;
    __MAP_ANGLE_TO_UNIT_CIRCLE(error_angle);

    // Apply low pass filter
    prev_error = LPF_ALPHA * error_angle + (1.0 - LPF_ALPHA) * prev_error;
    gimbal_angle_difference = prev_error;
    __MAP_ANGLE_TO_UNIT_CIRCLE(gimbal_angle_difference);
    
    // Rotate chassis toward snapped corner
    g_chassis_state.omega = PID(&angle_locking_pid, gimbal_angle_difference);
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
    float spin_coeff = chassis_rad * g_spintop_omega / (translation_speed * 25.0f + chassis_rad * g_spintop_omega);
    float target_omega = g_spintop_omega * spin_coeff * g_supercap_spintop_boost_rate;
    return target_omega;
}

/**
 * @brief returns the fastest drive speed of all the swerve modules
 * @return speed in m/s?
 */
float get_fastest_wheel_speed()
{
    float fastest_speed = 0.0f;
    for (int i = 0; i < NUMBER_OF_MODULES; i++)
    {
        float abs_velocity = fabsf(g_chassis_state.states[i].speed);
        if (abs_velocity > fastest_speed)
        {
            fastest_speed = abs_velocity;
        }
    }
    return fastest_speed;
}
