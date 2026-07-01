#include "chassis_task.h"

#include "motor.h"
#include "robot.h"
#include "remote.h"
#include "dji_motor.h"
#include "dm_motor.h"
#include "omni_locomotion.h"
#include "rate_limiter.h"
#include "pid.h"
#include "imu_task.h"
#include "jetson_orin.h"
#include <math.h>
#include "c_board_comm.h" 

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;
float gimbal_angle_difference;
pose_2d_t sentry_pose;
DJI_Motor_Handle_t *motors[4];
uint8_t drive_esc_id_array[4] = {1, 2, 3, 4};
Motor_Reversal_t drive_motor_reversal_array[4] = {
    MOTOR_REVERSAL_NORMAL,
    MOTOR_REVERSAL_NORMAL,
    MOTOR_REVERSAL_NORMAL,
    MOTOR_REVERSAL_NORMAL
};

extern DM_Motor_Handle_t *g_yaw; // for reading gimbal angle

omni_physical_constants_t physical_constants;
omni_chassis_state_t chassis_state;

extern Board_Comm_Package_t g_board_comm_package;

rate_limiter_t wheel_rate_limiters[4];
PID_t g_follow_gimbal_angle_pid;                      // chassis following gimbal

pose_2d_t sentry_pose;
motor_data_t motor_data_odom;

uint16_t last_hp;
uint16_t is_hit_counter = 0;

float omega = 1;
int omega_bool = 0;

float chassis_omega_new_target = 0;

void Chassis_Task_Init(){
    // Init chassis hardware
    Motor_Config_t drive_motor_config = {
        .can_bus = 1,
        .control_mode = VELOCITY_CONTROL,
        .velocity_pid = {
            .kp = 300.0f,
            .kf = 100.0f,
            .output_limit = M3508_MAX_CURRENT_INT,
            .integral_limit = 3000.0f,
    }};

    for (int i = 0; i < 4; i++) {
        // configure drive motor
        drive_motor_config.speed_controller_id = drive_esc_id_array[i];
        drive_motor_config.motor_reversal = drive_motor_reversal_array[i];
        motors[i] = DJI_Motor_Init(&drive_motor_config, M3508_PLANETARY);
        DJI_Motor_Set_Control_Mode(motors[i], VELOCITY_CONTROL);
    }

    sentry_pose.x = 0;
    sentry_pose.y = 0;

    pose_2d_t init_pose = {
        .x = INIT_X_POS,
        .y = INIT_Y_POS,
        .theta = INIT_THETA
    };

    physical_constants = omni_init( //TODO: Set the actual constants for standard
        CHASSIS_WHEEL_DIAMETER,
        CHASSIS_RADIUS,
        CHASSIS_MOUNTING_ANGLE,
        CHASSIS_MAX_SPEED,
        &init_pose
    );

    chassis_state.v_x = 0.0f;
    chassis_state.v_y = 0.0f;
    chassis_state.omega = 0.0f;

    for (int i = 0; i < 4; i++) {
        // configure rate limiters
        rate_limiter_init(&wheel_rate_limiters[i], MAX_ABC);
    }

    // Init PID
    PID_Init(&g_follow_gimbal_angle_pid, 30, 0, 10000, 2*PI*30, 0, 0);
    sentry_pose.x = 0;
    sentry_pose.y = 0;

    last_hp = Referee_System.Robot_State.Remaining_HP;
}

void Chassis_Process_Target_Velocity()
{    
    // chassis_state.v_x_in_gimbal = g_remote.controller.left_stick.x / 660.0f * 4.0f;
    // chassis_state.v_y_in_gimbal = g_remote.controller.left_stick.y / 660.0f * 4.0f;
    chassis_state.v_x_in_gimbal = g_robot_state.input.vx;
    chassis_state.v_y_in_gimbal = g_robot_state.input.vy;

    gimbal_angle_difference =  g_yaw->stats->pos;
    __MAP_ANGLE_TO_UNIT_CIRCLE(gimbal_angle_difference);

    chassis_omega_new_target = 0;
    const float hit_timeout = 5; // seconds

    // If the robot is hit, increase spintop rate
    if(Referee_System.Robot_State.Remaining_HP < last_hp) {
        is_hit_counter = 500 * hit_timeout;
    }

    //TODO：Adjust the data type to reflect the actual value.
    last_hp = Referee_System.Robot_State.Remaining_HP;
    
    // Counter for hit timeout
    if (is_hit_counter > 0) {
        is_hit_counter--;
    }

    if (g_robot_state.chassis.IS_SPINTOP_ENABLED /*|| g_remote.controller.left_switch == UP*/ ) {
        
        if (is_hit_counter > 0) {
            chassis_omega_new_target = 6 * PI; // 8 * PI rad/s
        }
        else {  //Decrease spintop rate if not hit for a while
            chassis_omega_new_target = omega; // 2 * PI rad/s
        }
        __FIRST_ORDER_FILTER(chassis_state.omega, chassis_omega_new_target, 0.001f);

        // float frenquncy = 0.1f;
        // speed_up_spintop_rate = 8 * PI * sin(2*PI*frenquncy*time_for_omega);
        // __FIRST_ORDER_FILTER(chassis_state.omega, chassis_omega_new_target, 0.001f);
       
    } else {
        __MAP_ANGLE_TO_UNIT_CIRCLE(gimbal_angle_difference);
        chassis_omega_new_target = PID(&g_follow_gimbal_angle_pid, gimbal_angle_difference);
        __MAX_LIMIT(chassis_omega_new_target, -6*2*PI, 6*2*PI);
        __FIRST_ORDER_FILTER(chassis_state.omega, chassis_omega_new_target, 0.001f);
    }
    Update_Omega();
}

void Chassis_Ctrl_Loop()
{
    //TODO: change this, for odom only
    Chassis_Process_Target_Velocity();

    if (g_robot_state.IS_SUPER_CAPACITOR_ENABLED) {
        physical_constants.max_speed = 5.0f;
        __FIRST_ORDER_FILTER(chassis_state.v_x, 2 * (chassis_state.v_x_in_gimbal * cos(gimbal_angle_difference) - chassis_state.v_y_in_gimbal * sin(gimbal_angle_difference)), 0.005f);
        __FIRST_ORDER_FILTER(chassis_state.v_y, 2 * (chassis_state.v_x_in_gimbal * sin(gimbal_angle_difference) + chassis_state.v_y_in_gimbal * cos(gimbal_angle_difference)), 0.005f);
    } else {
        physical_constants.max_speed = 2.0f;
        chassis_state.v_x = chassis_state.v_x_in_gimbal * cos(gimbal_angle_difference) - chassis_state.v_y_in_gimbal * sin(gimbal_angle_difference);
        chassis_state.v_y = chassis_state.v_x_in_gimbal * sin(gimbal_angle_difference) + chassis_state.v_y_in_gimbal * cos(gimbal_angle_difference);
    }

    // Control loop for the chassis
    omni_calculate_kinematics(&chassis_state, &physical_constants);
    omni_convert_to_rpm(&chassis_state);

    // use rate limiter to limit acceleration of the wheels
    // chassis_state.phi_dot_1 = rate_limiter_iterate(&wheel_rate_limiters[0], chassis_state.phi_dot_1);
    // chassis_state.phi_dot_2 = rate_limiter_iterate(&wheel_rate_limiters[1], chassis_state.phi_dot_2);
    // chassis_state.phi_dot_3 = rate_limiter_iterate(&wheel_rate_limiters[2], chassis_state.phi_dot_3);
    // chassis_state.phi_dot_4 = rate_limiter_iterate(&wheel_rate_limiters[3], chassis_state.phi_dot_4);

    // set the velocities of the wheels
    DJI_Motor_Set_Velocity(motors[0], chassis_state.phi_dot_1);
    DJI_Motor_Set_Velocity(motors[1], chassis_state.phi_dot_2);
    DJI_Motor_Set_Velocity(motors[2], chassis_state.phi_dot_3);
    DJI_Motor_Set_Velocity(motors[3], chassis_state.phi_dot_4);

    motor_data_odom.front_left = DJI_Motor_Get_Total_Angle(motors[0]) * physical_constants.R;
    motor_data_odom.back_left = DJI_Motor_Get_Total_Angle(motors[1]) * physical_constants.R;
    motor_data_odom.back_right = DJI_Motor_Get_Total_Angle(motors[2]) * physical_constants.R;
    motor_data_odom.front_right = DJI_Motor_Get_Total_Angle(motors[3]) * physical_constants.R;
}

void Update_Omega() 
{
    switch ((int) g_board_comm_package.Ps) {
        case 60:
            omega = OMEGA_55W;
            break;
        case 65:
            omega = OMEGA_65W;
            break;
        case 70:
            omega = OMEGA_70W;
            break;
        case 75:
            omega = OMEGA_75W;
            break;
        case 80:
            omega = OMEGA_80W;
            break;
        case 85:
            omega = OMEGA_85W;
            break;
        case 90:
            omega = OMEGA_90W;
            break;
        default:
            omega = OMEGA_55W;
    }

    if (g_robot_state.IS_SUPER_CAPACITOR_ENABLED) {
        omega = 20.0f;
        omega = Rescale_Omega();
    }
}

float Rescale_Omega(void) {
    float translation_speed = sqrtf(powf(g_remote.controller.left_stick.x / 660.0f * 4.0f, 2) + powf(g_remote.controller.left_stick.y / 660.0f * 4.0f, 2));
    float spin_coeff = CHASSIS_RADIUS * omega / (translation_speed * 500.0f + CHASSIS_RADIUS * omega);
    float target_omega = omega * spin_coeff;
    return target_omega;
}
