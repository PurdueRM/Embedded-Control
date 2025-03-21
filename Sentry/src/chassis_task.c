#include "chassis_task.h"

#include "motor.h"
#include "robot.h"
#include "remote.h"
#include "dji_motor.h"
#include "omni_locomotion.h"
#include "rate_limiter.h"

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;

DJI_Motor_Handle_t *motors[4];
uint8_t drive_esc_id_array[4] = {1, 2, 3, 4};
Motor_Reversal_t drive_motor_reversal_array[4] = {
    MOTOR_REVERSAL_REVERSED,
    MOTOR_REVERSAL_REVERSED,
    MOTOR_REVERSAL_REVERSED,
    MOTOR_REVERSAL_REVERSED
};

omni_physical_constants_t physical_constants;
omni_chassis_state_t chassis_state;

rate_limiter_t wheel_rate_limiters[4];

void Chassis_Task_Init()
{
    // Init chassis hardware
    Motor_Config_t drive_motor_config = {
        .can_bus = 1,
        .control_mode = VELOCITY_CONTROL,
        .velocity_pid = {
            .kp = 500.0f,
            .kf = 100.0f,
            .output_limit = M3508_MAX_CURRENT,
            .integral_limit = 3000.0f,
    }};

    for (int i = 0; i < 4; i++) {
        // configure drive motor
        drive_motor_config.speed_controller_id = drive_esc_id_array[i];
        drive_motor_config.motor_reversal = drive_motor_reversal_array[i];
        motors[i] = DJI_Motor_Init(&drive_motor_config, M3508);
        DJI_Motor_Set_Control_Mode(motors[i], VELOCITY_CONTROL);
    }

    // Initialize the omni chassis state
    physical_constants = omni_init(
        CHASSIS_WHEEL_DIAMETER,
        CHASSIS_RADIUS,
        CHASSIS_MOUNTING_ANGLE,
        CHASSIS_MAX_SPEED
    );

    chassis_state.v_x = 0.0f;
    chassis_state.v_y = 0.0f;
    chassis_state.omega = 0.0f;

    for (int i = 0; i < 4; i++) {
        // configure rate limiters
        rate_limiter_init(&wheel_rate_limiters[i], MAX_ABC);
    }

    
}

void Chassis_Ctrl_Loop()
{
    if (g_robot_state.chassis.IS_SPINTOP_ENABLED) {
        chassis_state.omega = SPIN_TOP_OMEGA;
    } else {
        //chassis_state.omega = g_robot_state.chassis.omega * MAX_ANGLUAR_SPEED;
        chassis_state.omega = g_robot_state.input.vomega;
    }
    
    chassis_state.v_x = g_robot_state.input.vy; // x and y are swapped due to joytick orientation
    chassis_state.v_y = -g_robot_state.input.vx;
    

    // Control loop for the chassis
    omni_calculate_kinematics(&chassis_state, &physical_constants);
    omni_desaturate_wheel_speeds(&chassis_state, &physical_constants);
    omni_convert_to_rpm(&chassis_state);

    // use rate limiter to limit acceleration of the wheels
    chassis_state.phi_dot_1 = rate_limiter_iterate(&wheel_rate_limiters[0], chassis_state.phi_dot_1);
    chassis_state.phi_dot_2 = rate_limiter_iterate(&wheel_rate_limiters[1], chassis_state.phi_dot_2);
    chassis_state.phi_dot_3 = rate_limiter_iterate(&wheel_rate_limiters[2], chassis_state.phi_dot_3);
    chassis_state.phi_dot_4 = rate_limiter_iterate(&wheel_rate_limiters[3], chassis_state.phi_dot_4);

    // set the velocities of the wheels
    DJI_Motor_Set_Velocity(motors[0], chassis_state.phi_dot_1);
    DJI_Motor_Set_Velocity(motors[1], chassis_state.phi_dot_2);
    DJI_Motor_Set_Velocity(motors[2], chassis_state.phi_dot_3);
    DJI_Motor_Set_Velocity(motors[3], chassis_state.phi_dot_4);
}


