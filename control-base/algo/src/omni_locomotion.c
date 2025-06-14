#include "omni_locomotion.h"

#include <math.h>
#include "user_math.h"

/**
 * @brief Initialize the physical constants of the omni drive
 * @param R - the wheel diameter [m]
 * @param d - the radius from the center to the wheel [m]
 * @param theta - the mounting angle of the omni wheel [rad]
 * @param max_speed - the maximum speed of the robot [m/s]
 * @param init_pose - The initial 2d x, y position and heading
 * @return omni_physical_constants_t - the physical constants of the omni drive
 */
omni_physical_constants_t omni_init(float R, float d, float theta, float max_speed, pose_2d_t* init_pose)
{
    // Initialize the physical constants of the omni drive
    omni_physical_constants_t omni_physical_constants = {
        .R = R,
        .d = d,
        .max_speed = max_speed,
        .init_pose = init_pose,
        .theta = theta,
        .kinematics_matrix = {
            {-sin(theta) / R,   -cos(theta) / R , d / R},
            {sin(theta) / R,    -cos(theta) / R , d / R},
            {sin(theta) / R,    cos(theta) / R  , d / R},
            {-sin(theta) / R,   cos(theta) / R  , d / R},
        },
    };
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
 * @brief Convert the chassis state from rad/s to rotations per minute
 * @param chassis_state: the chassis state to convert
 */
void omni_convert_to_rpm(omni_chassis_state_t *chassis_state) 
{
    chassis_state->phi_dot_1 *=  60.0f / (2.0f * PI);
    chassis_state->phi_dot_2 *=  60.0f / (2.0f * PI);
    chassis_state->phi_dot_3 *=  60.0f / (2.0f * PI);
    chassis_state->phi_dot_4 *=  60.0f / (2.0f * PI);
}

motor_data_t* prev = NULL;

/**
 * @brief Init the omni robot's odometry data
 * @param pose: The robot's pose data structure
 * @param init_x: The robot's initial 2d x position
 * @param init_y: The robot's initial 2d y position
 * @param init_theta: The robot's intial (gimbal) heading
 * @param motor_total_angles: Array of 4 motor angles
 */
void _Init_Omni_Odometry(pose_2d_t* pose, pose_2d_t* init_pose, motor_data_t* curr_motor_data)
{
    pose->x = init_pose->x;
    pose->y = init_pose->y;
    pose->theta = init_pose->theta;

    static motor_data_t prev_storage;
    prev = &prev_storage;

    prev->front_left = curr_motor_data->front_left;
    prev->back_left  = curr_motor_data->back_left;
    prev->back_right = curr_motor_data->back_right;
    prev->front_right= curr_motor_data->front_right;
}

/**
 * @brief Update the omni robot's odometry data
 */
void Update_Omni_Odometry(pose_2d_t* pose, omni_physical_constants_t* omni_physical_constants, motor_data_t* curr_motor_data, float heading_angle)
{
    if (prev != NULL)
    {
        motor_data_t motor_inc = {
            .front_left = curr_motor_data->front_left - prev->front_left,
            .back_left  = curr_motor_data->back_left  - prev->back_left,
            .back_right = curr_motor_data->back_right - prev->back_right,
            .front_right= curr_motor_data->front_right- prev->front_right
        };

        pose_2d_t pose_inc = {
            .x = (-motor_inc.front_left + motor_inc.back_left + motor_inc.back_right - motor_inc.front_right) / 4 * cos(PI/4),
            .y = (-motor_inc.front_left  - motor_inc.back_left + motor_inc.back_right + motor_inc.front_right) / 4 * sin(PI/4),
            .theta = heading_angle
        };

        pose->x += pose_inc.x * sin(heading_angle) - pose_inc.y * cos(heading_angle);
        pose->y += pose_inc.x * -cos(heading_angle) + pose_inc.y * sin(heading_angle);
        pose->theta = heading_angle;

        
        prev->front_left = curr_motor_data->front_left;
        prev->back_left  = curr_motor_data->back_left;
        prev->back_right = curr_motor_data->back_right;
        prev->front_right= curr_motor_data->front_right;
    }
    else
    {
        _Init_Omni_Odometry(pose, omni_physical_constants->init_pose, curr_motor_data);
    }
}