#include "swerve_locomotion.h"

#include <math.h>
#include "user_math.h"

/**
 * @brief Initializes the swerve drive constants.
 *
 * @param track_width The distance between the left and right wheels.
 * @param wheel_base The distance between the front and rear wheels.
 * @param wheel_diameter The diameter of the wheels.
 * @param max_speed The maximum speed of the swerve drive.
 * @param max_angular_speed The maximum angular speed of the swerve drive.
 * @return Initialized swerve constants.
 */
swerve_constants_t swerve_init(float track_width, float wheel_base, float wheel_diameter, float max_speed, float max_angular_speed) {
    swerve_constants_t swerve_constants = {
        .track_width = track_width,
        .wheel_base = wheel_base,
        .wheel_diameter = wheel_diameter,
        .max_speed = max_speed,
        .max_angular_speed = max_angular_speed,
        .kinematics_matrix = {
            {0, 1, -(wheel_base / 2)}, // front left
            {1, 0, -(track_width / 2)},
            {0, 1, -(wheel_base / 2)}, // rear left
            {1, 0, +(track_width / 2)},
            {0, 1, +(wheel_base / 2)}, // rear right
            {1, 0, +(track_width / 2)},
            {0, 1, +(wheel_base / 2)}, // front right
            {1, 0, -(track_width / 2)}
        }
    };

    return swerve_constants;
}

/**
 * @brief Calculates swerve drive kinematics.
 *
 * @param chassis_state Pointer to the chassis state structure.
 * @param swerve_constants Pointer to the swerve constants structure.
 */
void swerve_calculate_kinematics(swerve_chassis_state_t *chassis_state, swerve_constants_t *swerve_constants) {
    float v_x = chassis_state->v_x;
    float v_y = chassis_state->v_y;
    float omega = chassis_state->omega;
    float(*k_mat)[3] = swerve_constants->kinematics_matrix;

    if (v_x == 0 && v_y == 0 && omega == 0)
    {
        for (int i = 0; i < NUMBER_OF_MODULES; i++)
        {
            chassis_state->states[i].speed = 0;
            chassis_state->states[i].angle = chassis_state->states[i].last_angle;
        }
        return;
    }

    // Multiply the inverse kinematics matrix by the chassis speeds vector
    float module_states_matrix[8] = {
        k_mat[0][0] * v_x + k_mat[0][1] * v_y + k_mat[0][2] * omega,
        k_mat[1][0] * v_x + k_mat[1][1] * v_y + k_mat[1][2] * omega,
        k_mat[2][0] * v_x + k_mat[2][1] * v_y + k_mat[2][2] * omega,
        k_mat[3][0] * v_x + k_mat[3][1] * v_y + k_mat[3][2] * omega,
        k_mat[4][0] * v_x + k_mat[4][1] * v_y + k_mat[4][2] * omega,
        k_mat[5][0] * v_x + k_mat[5][1] * v_y + k_mat[5][2] * omega,
        k_mat[6][0] * v_x + k_mat[6][1] * v_y + k_mat[6][2] * omega,
        k_mat[7][0] * v_x + k_mat[7][1] * v_y + k_mat[7][2] * omega,
    };

    // Convert module x,y matrix to wheel speed and angle
    for (int i = 0; i < NUMBER_OF_MODULES; i++)
    {
        float x = module_states_matrix[i * 2 + 1];
        float y = module_states_matrix[i * 2];
        float speed = hypotf(x, y);

        chassis_state->states[i].speed = speed;

        if (speed > 1e-6f)
        {
            float angle = atan2f(y, x);
            chassis_state->states[i].angle = angle;
            chassis_state->states[i].last_angle = angle;
        }
        else
        {
            chassis_state->states[i].angle = chassis_state->states[i].last_angle;
        }
    }
}

/**
 * @brief Desaturates wheel speeds to ensure they do not exceed the max speed.
 *
 * @param chassis_state Pointer to the chassis state structure.
 * @param swerve_constants Pointer to the swerve constants structure.
 */
void swerve_desaturate_wheel_speeds(swerve_chassis_state_t *chassis_state, swerve_constants_t *swerve_constants) {
    float highest_speed = fabsf(chassis_state->states[0].speed);
    for (int i = 1; i < NUMBER_OF_MODULES; i++) // start from 1 to find the highest speed
    {
        highest_speed = fmaxf(highest_speed, fabsf(chassis_state->states[i].speed));
    }

    if (highest_speed < 0.01f) // avoid division by zero
    {
        return;
    }

    if (highest_speed > swerve_constants->max_speed) {
        float desaturation_coefficient = fabsf(swerve_constants->max_speed / highest_speed);
        for (int i = 0; i < NUMBER_OF_MODULES; i++)
        {
            chassis_state->states[i].speed = chassis_state->states[i].speed * desaturation_coefficient;
        }
    }
}


/**
 * @brief Optimizes the angles of the swerve modules to minimize rotation.
 *
 * @param chassis_state Pointer to the swerve chassis state structure.
 * @param measured_angles Array of measured angles for each module.
 */
void swerve_optimize_module_angles(swerve_chassis_state_t *chassis_state, float measured_angles[NUMBER_OF_MODULES]) {
    for (int i = 0; i < NUMBER_OF_MODULES; i++)
    {
        if (measured_angles[i] > PI) {
            measured_angles[i] -= 2 * PI;
        }

        float angle_diff = chassis_state->states[i].angle - measured_angles[i];

        if (angle_diff >= PI_OVER_2 && angle_diff <= 3 * PI_OVER_2) {
            chassis_state->states[i].angle -= PI;
            chassis_state->states[i].speed *= -1.0f;
        }
        else if (angle_diff <= -PI_OVER_2 && angle_diff >= -3 * PI_OVER_2) {
            chassis_state->states[i].angle += PI;
            chassis_state->states[i].speed *= -1.0f;
        }

        if (chassis_state->states[i].angle < 0) {
            chassis_state->states[i].angle += 2 * PI;
        }
    }
}

/**
 * Convert the chassis state from m/s to rotations per minute
 * @param chassis_state: the chassis state to convert
 * @param wheel_diameter: the diameter of the wheel
 */
 void swerve_convert_to_rpm(swerve_chassis_state_t *chassis_state, swerve_constants_t *swerve_constants) {
    for (int i = 0; i < NUMBER_OF_MODULES; i++) {
        chassis_state->states[i].speed *= 60 / (PI * swerve_constants->wheel_diameter);
    }
}
