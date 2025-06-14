#include "chassis_task.h"

#include "robot.h"
#include "remote.h"
#include "dm_motor.h"
#include "two_bar_leg.h"
#include "pid.h"
#include "user_math.h"
#include "wheel_legged_3d_lqr.h"
#include "wheel_legged_2d_lqr.h"
#include "dji_motor.h"
#include "imu_task.h"

#define LEFT_THIGH (1)
#define LEFT_CALF (0)
#define RIGHT_THIGH (3)
#define RIGHT_CALF (2)

extern Robot_State_t g_robot_state;
extern Remote_t g_remote;
DM_Motor_Handle_t *g_chassis_joint_motor[4] = {NULL};
DJI_Motor_Handle_t *g_chassis_drive_motor_left = NULL;
DJI_Motor_Handle_t *g_chassis_drive_motor_right = NULL;
Two_Bar_Kinematics_t g_right_leg_kinematics = {0};
Two_Bar_Kinematics_t g_left_leg_kinematics = {0};
PID_t g_chassis_right_leg_length_pid = {0};
PID_t g_chassis_left_leg_length_pid = {0};
PID_t g_chassis_right_leg_angle_pid = {0};
PID_t g_chassis_left_leg_angle_pid = {0};
Two_Bar_Virtual_Force right_virtual_force = {
    .supportive_force = 0,
    .torque = 0.0f};
Two_Bar_Motor_Torque right_motor_torque = {
    .torque1 = 0.0f,
    .torque2 = 0.0f};

Two_Bar_Virtual_Force left_virtual_force = {
    .supportive_force = 0,
    .torque = 0.0f};

Two_Bar_Motor_Torque left_motor_torque = {
    .torque1 = 0.0f,
    .torque2 = 0.0f};
float g_chassis_left_drive_torque = 0.0f;
float g_chassis_right_drive_torque = 0.0f;
struct
{
    float target_left_leg_length;
    float target_left_leg_angle;
    float target_right_leg_length;
    float target_right_leg_angle;
} g_chassis_target;

float g_chassis_right_angle_error;
float g_chassis_right_angle_error_original;

WheelLeggedState g_wheel_legged_state = {0};
WheelLeggedInput g_wheel_legged_input = {0};
lqr_ss_t g_2d_left_state = {0};
lqr_ss_t g_2d_right_state = {0};
lqr_u_t g_2d_left_input = {0};
lqr_u_t g_2d_right_input = {0};

void Chassis_Task_Init()
{
    DM_Motor_Config_t motor_config = {
        .can_bus = 1,
        .control_mode = DM_MOTOR_MIT,
        .disable_behavior = DM_MOTOR_HARDWARE_DISABLE, // DM_MOTOR_HARDWARE_DISABLE
        .pos_offset = 0.0f,
        .kp = 10.0f,
        .kd = .5f,
    };
    // 0x0 = -1.28372097, 0x01= -0.781379461 left datui,  
    // 0x2 = 1.962116, 0x3 = 4.57898361 right datui
    // 0x1: -0.67654705, 0x3: -1.28080368, 0x0: -1.69623089 , 0x2: -0.658999443
    float pos_offset[4] = {1.3323f, -0.67654705f, -0.658999443f, 1.9549f};
    for (int i = 0; i < 4; i++)
    {
        motor_config.tx_id = 0x00 + i;
        motor_config.rx_id = 0x10 + i;
        motor_config.pos_offset = pos_offset[i];
        g_chassis_joint_motor[i] = DM_Motor_Init(&motor_config);
    }
    Motor_Config_t drive_motor_config = {
        .can_bus = 1,
        .speed_controller_id = 0x01,
        .motor_reversal = MOTOR_REVERSAL_NORMAL,
        .control_mode = TORQUE_CONTROL,
    };
    g_chassis_drive_motor_right = DJI_Motor_Init(&drive_motor_config, M3508_PLANETARY);
    drive_motor_config.motor_reversal = MOTOR_REVERSAL_REVERSED;
    drive_motor_config.speed_controller_id = 0x02;
    g_chassis_drive_motor_left = DJI_Motor_Init(&drive_motor_config, M3508_PLANETARY);
    
    PID_Init(&g_chassis_right_leg_length_pid, 100.0f, 0.0f, 1000.0f, 50.0f, 0.0f, 0.0f);
    PID_Init(&g_chassis_left_leg_length_pid, 100.0f, 0.0f, 1000.0f, 50.0f, 0.0f, 0.0f);
    PID_Init(&g_chassis_right_leg_angle_pid, 20.0f, 0.0f, 200.0f, 40.0f, 0.0f, 0.0f);
    PID_Init(&g_chassis_left_leg_angle_pid, 20.0f, 0.0f, 200.0f, 40.0f, 0.0f, 0.0f);


}

float Chassis_Get_Wheel_Displacement()
{
    // Calculate the wheel displacement based on the chassis state
    float wheel_angle_displacement = (DJI_Motor_Get_Total_Angle(g_chassis_drive_motor_left) + DJI_Motor_Get_Total_Angle(g_chassis_drive_motor_right)) / 2.0f;
    float wheel_displacement = wheel_angle_displacement * 0.116f/2;
    return wheel_displacement;
}

void Chassis_State_Estimation(float dt, const float filter_constant, float phi, float dphi, float theta_b, float dtheta_b)
{
    Two_Bar_Forward_Kinematics(&g_right_leg_kinematics, g_chassis_joint_motor[RIGHT_THIGH]->stats->pos, g_chassis_joint_motor[RIGHT_CALF]->stats->pos);
    Two_Bar_Forward_Kinematics(&g_left_leg_kinematics, g_chassis_joint_motor[LEFT_THIGH]->stats->pos, g_chassis_joint_motor[LEFT_CALF]->stats->pos);

    // Construct Wheel Legged State
    g_wheel_legged_state.s = Chassis_Get_Wheel_Displacement() + g_wheel_legged_state.s_offset;
    float ds_tmp = (g_wheel_legged_state.s - g_wheel_legged_state.s_prev) / dt;
    g_wheel_legged_state.ds = filter_constant * ds_tmp + (1.0f - filter_constant) * g_wheel_legged_state.ds;
    g_wheel_legged_state.s_prev = g_wheel_legged_state.s;

    g_wheel_legged_state.phi = phi;
    g_wheel_legged_state.dphi = dphi;
    
    g_wheel_legged_state.theta_ll = (g_left_leg_kinematics.theta + PI/2.0f);
    float dtheta_ll_tmp = (g_left_leg_kinematics.theta - g_wheel_legged_state.theta_ll_prev) / dt;
    g_wheel_legged_state.dtheta_ll = filter_constant * dtheta_ll_tmp + (1.0f - filter_constant) * g_wheel_legged_state.dtheta_ll;
    g_wheel_legged_state.theta_ll_prev = g_left_leg_kinematics.theta;

    g_wheel_legged_state.theta_lr = -(g_right_leg_kinematics.theta + PI/2.0f);
    float dtheta_lr_tmp = (g_right_leg_kinematics.theta - g_wheel_legged_state.theta_lr_prev) / dt;
    g_wheel_legged_state.dtheta_lr = filter_constant * dtheta_lr_tmp + (1.0f - filter_constant) * g_wheel_legged_state.dtheta_lr;
    g_wheel_legged_state.theta_lr_prev = g_right_leg_kinematics.theta;

    g_wheel_legged_state.theta_b = theta_b;
    g_wheel_legged_state.dtheta_b = dtheta_b;

    g_2d_left_state.x = g_wheel_legged_state.s;
    g_2d_left_state.x_dot = g_wheel_legged_state.ds;
    g_2d_left_state.theta = g_wheel_legged_state.theta_ll;
    g_2d_left_state.theta_dot = g_wheel_legged_state.dtheta_ll;
    g_2d_left_state.phi = g_wheel_legged_state.theta_b;
    g_2d_left_state.phi_dot = g_wheel_legged_state.dtheta_b;
    g_2d_left_state.leg_len = g_left_leg_kinematics.leg_length;

    g_2d_right_state.x = g_wheel_legged_state.s;
    g_2d_right_state.x_dot = g_wheel_legged_state.ds;
    g_2d_right_state.theta = g_wheel_legged_state.theta_lr;
    g_2d_right_state.theta_dot = g_wheel_legged_state.dtheta_lr;
    g_2d_right_state.phi = g_wheel_legged_state.theta_b;
    g_2d_right_state.phi_dot = g_wheel_legged_state.dtheta_b;
    g_2d_right_state.leg_len = g_right_leg_kinematics.leg_length;
}

void Chassis_LQR(float leg_length, float feedforward_weight_force)
{
    g_chassis_target.target_left_leg_length = leg_length + g_remote.controller.wheel / 3000.0f;
    g_chassis_target.target_right_leg_length = leg_length + g_remote.controller.wheel / 3000.0f;
    Wheel_Legged_Compute_LQR_output(&g_wheel_legged_state, g_left_leg_kinematics.leg_length, g_right_leg_kinematics.leg_length, &g_wheel_legged_input);
    
    // Leg Length PD
    left_virtual_force.supportive_force = PID(&g_chassis_left_leg_length_pid, g_chassis_target.target_left_leg_length - g_left_leg_kinematics.leg_length) + feedforward_weight_force;
    right_virtual_force.supportive_force = PID(&g_chassis_right_leg_length_pid, g_chassis_target.target_right_leg_length - g_right_leg_kinematics.leg_length) + feedforward_weight_force;

    // Apply LQR Input
    left_virtual_force.torque = g_wheel_legged_input.T_bl;
    right_virtual_force.torque = -g_wheel_legged_input.T_br;

    /***** FOR DEBUG Leg Angle Start *****/
    // g_chassis_target.target_left_leg_angle = g_remote.controller.wheel / 110.0f * 0 - 1.57f;
    // g_chassis_target.target_right_leg_angle = g_remote.controller.wheel / 110.0f * 0 - 1.57f;

    // // Right Leg
    // float angle_error = g_chassis_target.target_right_leg_angle - g_right_leg_kinematics.theta;
    // __MAP_ANGLE_TO_UNIT_CIRCLE(angle_error);
    // right_virtual_force.torque = PID(&g_chassis_right_leg_angle_pid, angle_error);

    // // Left Leg
    // angle_error = g_chassis_target.target_left_leg_angle - g_left_leg_kinematics.theta;
    // __MAP_ANGLE_TO_UNIT_CIRCLE(angle_error);
    // left_virtual_force.torque = PID(&g_chassis_left_leg_angle_pid, angle_error);

    g_chassis_left_drive_torque = g_wheel_legged_input.T_wl;
    g_chassis_right_drive_torque = -g_wheel_legged_input.T_wr;
    __MAX_LIMIT(g_chassis_left_drive_torque, -M3508_MAX_CURRENT*M3508_PLANETARY_TORQUE_CONSTANT, M3508_MAX_CURRENT*M3508_PLANETARY_TORQUE_CONSTANT);
    __MAX_LIMIT(g_chassis_right_drive_torque, -M3508_MAX_CURRENT*M3508_PLANETARY_TORQUE_CONSTANT, M3508_MAX_CURRENT*M3508_PLANETARY_TORQUE_CONSTANT);
    /***** FOR DEBUG Leg Angle End *****/

    // Forward Dynamics
    Two_Bar_Get_Motor_Torque_From_Virtual_Force(&g_right_leg_kinematics, &right_virtual_force, &right_motor_torque);
    Two_Bar_Get_Motor_Torque_From_Virtual_Force(&g_left_leg_kinematics, &left_virtual_force, &left_motor_torque);
}

void Chassis_LQR_2D(float leg_length, float feedforward_weight_force)
{
    g_chassis_target.target_left_leg_length = leg_length + g_remote.controller.wheel / 3000.0f;
    g_chassis_target.target_right_leg_length = leg_length + g_remote.controller.wheel / 3000.0f;
    
    LQR_Output(&g_2d_left_input, &g_2d_left_state);
    LQR_Output(&g_2d_right_input, &g_2d_right_state);

    // Leg Length PD
    left_virtual_force.supportive_force = PID(&g_chassis_left_leg_length_pid, 
        g_chassis_target.target_left_leg_length - g_left_leg_kinematics.leg_length) + feedforward_weight_force;
    right_virtual_force.supportive_force = PID(&g_chassis_right_leg_length_pid, 
        g_chassis_target.target_right_leg_length - g_right_leg_kinematics.leg_length) + feedforward_weight_force;

    // Apply LQR Input
    left_virtual_force.torque = g_2d_left_input.T_B;
    right_virtual_force.torque = -g_2d_right_input.T_B;

    /***** FOR DEBUG Leg Angle Start *****/
    g_chassis_target.target_left_leg_angle = g_remote.controller.wheel / 110.0f * 0 - 1.57f;
    g_chassis_target.target_right_leg_angle = g_remote.controller.wheel / 110.0f * 0 - 1.57f;

    // // Right Leg
    float angle_error = g_chassis_target.target_right_leg_angle - g_right_leg_kinematics.theta;
    __MAP_ANGLE_TO_UNIT_CIRCLE(angle_error);
    right_virtual_force.torque = PID(&g_chassis_right_leg_angle_pid, angle_error);

    // // Left Leg
    angle_error = g_chassis_target.target_left_leg_angle - g_left_leg_kinematics.theta;
    __MAP_ANGLE_TO_UNIT_CIRCLE(angle_error);
    left_virtual_force.torque = PID(&g_chassis_left_leg_angle_pid, angle_error);
    Two_Bar_Get_Motor_Torque_From_Virtual_Force(&g_right_leg_kinematics, &right_virtual_force, &right_motor_torque);
    Two_Bar_Get_Motor_Torque_From_Virtual_Force(&g_left_leg_kinematics, &left_virtual_force, &left_motor_torque);

    g_chassis_left_drive_torque = g_2d_left_input.T_A;
    g_chassis_right_drive_torque = g_2d_right_input.T_A;
    __MAX_LIMIT(g_chassis_left_drive_torque, -M3508_MAX_CURRENT*M3508_PLANETARY_TORQUE_CONSTANT, M3508_MAX_CURRENT*M3508_PLANETARY_TORQUE_CONSTANT);
    __MAX_LIMIT(g_chassis_right_drive_torque, -M3508_MAX_CURRENT*M3508_PLANETARY_TORQUE_CONSTANT, M3508_MAX_CURRENT*M3508_PLANETARY_TORQUE_CONSTANT);
}

void Chassis_LQR_2D_Right(float leg_length, float feedforward_weight_force)
{
    // g_chassis_target.target_left_leg_length = leg_length + g_remote.controller.wheel / 3000.0f;
    g_chassis_target.target_right_leg_length = leg_length + g_remote.controller.wheel / 3000.0f;
    
    // LQR_Output(&g_2d_left_input, &g_2d_left_state);
    LQR_Output(&g_2d_right_input, &g_2d_right_state);

    // Leg Length PD
    // left_virtual_force.supportive_force = PID(&g_chassis_left_leg_length_pid, 
    //     g_chassis_target.target_left_leg_length - g_left_leg_kinematics.leg_length) + feedforward_weight_force;
    right_virtual_force.supportive_force = PID(&g_chassis_right_leg_length_pid, 
        g_chassis_target.target_right_leg_length - g_right_leg_kinematics.leg_length) + feedforward_weight_force;

    // Apply LQR Input
    // left_virtual_force.torque = g_2d_left_input.T_B;
    right_virtual_force.torque = -g_2d_right_input.T_B;

    // g_chassis_left_drive_torque = g_2d_left_input.T_A;
    g_chassis_right_drive_torque = g_2d_right_input.T_A;
    // __MAX_LIMIT(g_chassis_left_drive_torque, -M3508_MAX_CURRENT*M3508_PLANETARY_TORQUE_CONSTANT, M3508_MAX_CURRENT*M3508_PLANETARY_TORQUE_CONSTANT);
    __MAX_LIMIT(g_chassis_right_drive_torque, -M3508_MAX_CURRENT*M3508_PLANETARY_TORQUE_CONSTANT, M3508_MAX_CURRENT*M3508_PLANETARY_TORQUE_CONSTANT);
}

void Chassis_Set_Leg_And_Angle_By_Remote(float leg_length, float feedforward_weight_force)
{
    // Set Goal
        g_chassis_target.target_left_leg_length = leg_length + g_remote.controller.wheel / 3000.0f;
        g_chassis_target.target_left_leg_angle = g_remote.controller.wheel / 110.0f * 0 - 1.57f;
        g_chassis_target.target_right_leg_length = leg_length + g_remote.controller.wheel / 3000.0f;
        g_chassis_target.target_right_leg_angle = g_remote.controller.wheel / 110.0f * 0 - 1.57f;

        // Right Leg
        float virtual_supportive_force = PID(&g_chassis_right_leg_length_pid, g_chassis_target.target_right_leg_length - g_right_leg_kinematics.leg_length);
        float angle_error = g_chassis_target.target_right_leg_angle - g_right_leg_kinematics.theta;
        __MAP_ANGLE_TO_UNIT_CIRCLE(angle_error);
        right_virtual_force.supportive_force = virtual_supportive_force + feedforward_weight_force;
        right_virtual_force.torque = PID(&g_chassis_right_leg_angle_pid, angle_error);

        // Left Leg
        virtual_supportive_force = PID(&g_chassis_left_leg_length_pid, g_chassis_target.target_left_leg_length - g_left_leg_kinematics.leg_length);
        angle_error = g_chassis_target.target_left_leg_angle - g_left_leg_kinematics.theta;
        __MAP_ANGLE_TO_UNIT_CIRCLE(angle_error);
        left_virtual_force.supportive_force = virtual_supportive_force + feedforward_weight_force;
        left_virtual_force.torque = PID(&g_chassis_left_leg_angle_pid, angle_error);

        Two_Bar_Get_Motor_Torque_From_Virtual_Force(&g_right_leg_kinematics, &right_virtual_force, &right_motor_torque);
        Two_Bar_Get_Motor_Torque_From_Virtual_Force(&g_left_leg_kinematics, &left_virtual_force, &left_motor_torque);
}

void Chassis_Reset_State()
{
    g_wheel_legged_state.s_offset = -Chassis_Get_Wheel_Displacement();
}

void Chassis_Ctrl_Loop()
{
    Chassis_State_Estimation(0.002f, 0.2f, g_imu.rad.yaw, g_imu.bmi088_raw.gyro[IMU_BMI088_YAW_IDX],
        g_imu.rad.pitch, (g_imu.bmi088_raw.gyro[IMU_BMI088_PITCH_IDX]));
    Chassis_LQR_2D(0.20f, 10.0f);
    if (g_robot_state.state == ENABLED)
    {
        DM_Motor_Enable_Motor(g_chassis_joint_motor[0]);
        DM_Motor_Enable_Motor(g_chassis_joint_motor[1]);
        DM_Motor_Enable_Motor(g_chassis_joint_motor[2]);
        DM_Motor_Enable_Motor(g_chassis_joint_motor[3]);
    }
    // 

    if (g_remote.controller.right_switch == MID) // Open Loop VMC (Debug Only)
    {
        Chassis_Set_Leg_And_Angle_By_Remote(0.20f, 10.0f);
        // 0 and 2 are both xiaotui, 1 and 3 are both datui
        DM_Motor_Ctrl_MIT_PD(g_chassis_joint_motor[1], 0.0f, 0.0f, left_motor_torque.torque1, 0.0f, 0.0f);
        DM_Motor_Ctrl_MIT_PD(g_chassis_joint_motor[0], 0.0f, 0.0f, left_motor_torque.torque2, 0.0f, 0.0f);
        DM_Motor_Ctrl_MIT_PD(g_chassis_joint_motor[3], 0.0f, 0.0f, right_motor_torque.torque1, 0.0f, 0.0f);
        DM_Motor_Ctrl_MIT_PD(g_chassis_joint_motor[2], 0.0f, 0.0f, right_motor_torque.torque2, 0.0f, 0.0f);
        DJI_Motor_Set_Torque(g_chassis_drive_motor_left, g_remote.controller.left_stick.y / 660.0f * 3.6f);
        DJI_Motor_Set_Torque(g_chassis_drive_motor_right, g_remote.controller.right_stick.y / 660.0f * 3.6f);
    }
    else if (g_remote.controller.right_switch == UP) // Close Loop
    {
        Chassis_Set_Leg_And_Angle_By_Remote(0.20f, 10.0f);
        Chassis_LQR_2D(0.20f, 10.0f);

        // Apply to Hardware
        DM_Motor_Ctrl_MIT_PD(g_chassis_joint_motor[1], 0.0f, 0.0f, left_motor_torque.torque1, 0.0f, 0.0f);
        DM_Motor_Ctrl_MIT_PD(g_chassis_joint_motor[0], 0.0f, 0.0f, left_motor_torque.torque2, 0.0f, 0.0f);
        DM_Motor_Ctrl_MIT_PD(g_chassis_joint_motor[3], 0.0f, 0.0f, right_motor_torque.torque1, 0.0f, 0.0f);
        DM_Motor_Ctrl_MIT_PD(g_chassis_joint_motor[2], 0.0f, 0.0f, right_motor_torque.torque2, 0.0f, 0.0f);
        DJI_Motor_Set_Torque(g_chassis_drive_motor_left, g_chassis_left_drive_torque);
        DJI_Motor_Set_Torque(g_chassis_drive_motor_right, g_chassis_right_drive_torque);
    }


}

void Chassis_Task_Disable()
{
    Chassis_Reset_State();
}