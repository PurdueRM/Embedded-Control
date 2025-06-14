#include "two_bar_leg.h"
#include "arm_math.h"


Two_Bar_Config_t two_bar_config = {
    .thigh_length = 0.2f,
    .calf_length = 0.245f
};

void Two_Bar_Init(float thigh_length, float calf_length) {
    two_bar_config.thigh_length = thigh_length;
    two_bar_config.calf_length = calf_length;
}

void Two_Bar_Forward_Kinematics(Two_Bar_Kinematics_t *kinematics, float theta1, float theta2) {
    float L1 = two_bar_config.thigh_length;
    float L2 = two_bar_config.calf_length;

    kinematics->theta1 = theta1;
    kinematics->theta2 = theta2;

    // Calculate end effector position
    float x_end = L1 * arm_cos_f32(theta1) + L2 * arm_cos_f32(theta2);
    float y_end = L1 * arm_sin_f32(theta1) + L2 * arm_sin_f32(theta2);
    kinematics->theta = atan2f(y_end, x_end);
    // Calculate leg length
    kinematics->leg_length = sqrtf(x_end * x_end + y_end * y_end);
}

void Two_Bar_Get_Motor_Torque_From_Virtual_Force(Two_Bar_Kinematics_t *kinematics, Two_Bar_Virtual_Force *virtual_force, Two_Bar_Motor_Torque *motor_torque) {
    float L1 = two_bar_config.thigh_length;
    float L2 = two_bar_config.calf_length;
    float theta1 = kinematics->theta1;
    float theta2 = kinematics->theta2;
    float theta = kinematics->theta;
    float L = kinematics->leg_length;

    // fx = supportive_force * np.cos(theta) - torque / L * np.sin(theta)
    // fy = supportive_force * np.sin(theta) + torque / L * np.cos(theta)

    // torque1_xy = -L1 * np.sin(theta1) * fx + L1 * np.cos(theta1) * fy
    // torque2_xy = -L2 * np.sin(theta2) * fx + L2 * np.cos(theta2) * fy
    
    float fx = virtual_force->supportive_force * arm_cos_f32(theta) - virtual_force->torque / L * arm_sin_f32(theta);
    float fy = virtual_force->supportive_force * arm_sin_f32(theta) + virtual_force->torque / L * arm_cos_f32(theta);
    motor_torque->torque1 = -L1 * arm_sin_f32(theta1) * fx + L1 * arm_cos_f32(theta1) * fy;
    motor_torque->torque2 = -L2 * arm_sin_f32(theta2) * fx + L2 * arm_cos_f32(theta2) * fy;

    // // Calculate Jacobian matrix
    // float J11 = arm_sin_f32(theta + theta1) * L1;
    // float J12 = -arm_cos_f32(theta + theta1) * L1 / L;
    // float J21 = arm_sin_f32(theta + theta2) * L2;
    // float J22 = -arm_cos_f32(theta + theta2) * L2 / L;

    // Calculate Tau = J * [F Tau] Transpose
    // motor_torque->torque1 = J11 * virtual_force->supportive_force + J12 * virtual_force->torque;
    // motor_torque->torque2 = J21 * virtual_force->supportive_force + J22 * virtual_force->torque;
}