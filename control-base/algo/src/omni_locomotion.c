#include "omni_locomotion.h"

#include <math.h>
#include "user_math.h"

/**
 * @brief Initialize the physical constants of the omni drive
 * @param R - the wheel diameter [m]
 * @param d - the radius from the center to the wheel [m]
 * @param theta - the mounting angle of the omni wheel [rad]
 * @param max_speed - the maximum speed of the robot [m/s]
 * @return omni_physical_constants_t - the physical constants of the omni drive
 */
omni_physical_constants_t omni_init(float R, float d, float theta, float max_speed)
{
    // Initialize the physical constants of the omni drive
    omni_physical_constants_t omni_physical_constants = {
        .R = R,
        .d = d,
        .max_speed = max_speed,
        .kinematics_matrix = {
            {-sin(theta) / R, cos(theta) / R, d / R},
            {-cos(theta) / R, -sin(theta) / R, d / R},
            {sin(theta) / R, -cos(theta) / R, d / R},
            {cos(theta) / R, sin(theta) / R, d / R}}};

    return omni_physical_constants;
}

/**
 * @brief Calculate the wheel velocities based on the chassis velocities
 * @param input - the input state of the omni chassis
 * @param omni_physical_constants - the physical constants of the omni drive
 */
void omni_calculate_kinematics(omni_chassis_state_t *input, omni_physical_constants_t *omni_physical_constants)
{
    float v_x = input->v_x;
    float v_y = input->v_y;
    float omega = input->omega;
    float(*k_mat)[3] = omni_physical_constants->kinematics_matrix;
    // Calculate the wheel velocities by multiplying the IK matrix by the chassis velocities
    input->phi_dot_1 = k_mat[0][0] * v_x + k_mat[0][1] * v_y + k_mat[0][2] * omega;
    input->phi_dot_2 = k_mat[1][0] * v_x + k_mat[1][1] * v_y + k_mat[1][2] * omega;
    input->phi_dot_3 = k_mat[2][0] * v_x + k_mat[2][1] * v_y + k_mat[2][2] * omega;
    input->phi_dot_4 = k_mat[3][0] * v_x + k_mat[3][1] * v_y + k_mat[3][2] * omega;
}

/**
 * @brief Scale down the wheel speeds if they exceed the maximum speed
 * @param input - the input state of the omni chassis
 * @param omni_physical_constants - the physical constants of the omni drive
 */
void omni_desaturate_wheel_speeds(omni_chassis_state_t *input, omni_physical_constants_t *omni_physical_constants)
{
    // find the highest wheel speed
    float highest_wheel_speed = fabsf(input->phi_dot_1);
    highest_wheel_speed = fmaxf(highest_wheel_speed, fabsf(input->phi_dot_2));
    highest_wheel_speed = fmaxf(highest_wheel_speed, fabsf(input->phi_dot_3));
    highest_wheel_speed = fmaxf(highest_wheel_speed, fabsf(input->phi_dot_4));

    float max_angular_velocity = omni_physical_constants->max_speed / omni_physical_constants->R;

    // scale down the wheel speeds if they exceed the maximum speed
    if (fabsf(highest_wheel_speed) > max_angular_velocity)
    {
        float desaturation_coefficient = fabsf(max_angular_velocity / highest_wheel_speed);
        input->phi_dot_1 *= desaturation_coefficient;
        input->phi_dot_2 *= desaturation_coefficient;
        input->phi_dot_3 *= desaturation_coefficient;
        input->phi_dot_4 *= desaturation_coefficient;
    }
}

/**
 * Convert the chassis state from rad/s to rotations per minute
 * @param chassis_state: the chassis state to convert
 */
void omni_convert_to_rpm(omni_chassis_state_t *chassis_state) {
    chassis_state->phi_dot_1 *=  60.0f / (2.0f * PI);
    chassis_state->phi_dot_2 *=  60.0f / (2.0f * PI);
    chassis_state->phi_dot_3 *=  60.0f / (2.0f * PI);
    chassis_state->phi_dot_4 *=  60.0f / (2.0f * PI);
}