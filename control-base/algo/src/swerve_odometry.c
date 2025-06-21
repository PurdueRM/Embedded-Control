#include "swerve_odometry.h"

#include "swerve_locomotion.h"
#include "vec_utils.h"

#include <math.h>

float module_delta_data[NUMBER_OF_MODULES * 2];
float chassis_delta_data[3];

Mat module_deltas = {NUMBER_OF_MODULES * 2, 1, module_delta_data};
Mat chassis_delta = {3, 1, chassis_delta_data};

/**
 * @brief Calculates the forward kinematics of the swerve drive.
 *
 * @param pose Pointer to the pose structure.
 * @param wheel_states Array of module states.
 * @param swerve_constants Pointer to the swerve constants structure.
 * @param yaw Current yaw angle (rad) according to the IMU
 * @param previous_yaw Previous yaw angle (rad) according to the IMU (When the last update was called)
 * @param dt Time since the last update (s)
 */
void swerve_odometry_update(pose_2d *pose, module_state_t wheel_states[NUMBER_OF_MODULES], swerve_constants_t *swerve_constants,
            float yaw, float previous_yaw, float dt) {

    // Populate module deltas matrix
    for (int i = 0; i < NUMBER_OF_MODULES; i++) {
        module_deltas.data[i * 2] = wheel_states[i].speed * dt * cosf(wheel_states[i].angle);
        module_deltas.data[i * 2 + 1] = wheel_states[i].speed * dt * sinf(wheel_states[i].angle);
    }

    // Calculate chassis delta
    mat_mult_buffer(swerve_constants->forward_kinematics_matrix, &module_deltas, &chassis_delta);

    // Trust the IMU for yaw more than odometry
    chassis_delta.data[2] = yaw - previous_yaw;

    // Calculate effect of chassis delta on pose
    // Derivation can be found in section 10.4: https://file.tavsys.net/control/controls-engineering-in-frc.pdf
    float sin_theta = sinf(chassis_delta.data[2]);
    float cos_theta = cosf(chassis_delta.data[2]);

    float s;
    float c;
    if (fabsf(chassis_delta.data[2] < 1E-9)) {
        s = 1.0 - chassis_delta.data[2] * chassis_delta.data[2] / 6.0;
        c = 0.5 * chassis_delta.data[2];
    } else {
        s = sin_theta / chassis_delta.data[2];
        c = (1.0 - cos_theta) / chassis_delta.data[2];
    }

    // Apply delta to the pose
    pose->x += chassis_delta.data[0] * s - chassis_delta.data[1] * c;
    pose->y += chassis_delta.data[0] * c + chassis_delta.data[1] * s;
    pose->theta = yaw; // Pull directly from IMU
}