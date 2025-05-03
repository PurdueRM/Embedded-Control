#ifndef SWERVE_ODOMETRY_H
#define SWERVE_ODOMETRY_H

#include "swerve_locomotion.h"

typedef struct {
    float x; // x position in meters
    float y; // y position in meters
    float theta; // orientation in radians
} pose_2d;

void swerve_odometry_update(pose_2d *pose, module_state_t wheel_states[NUMBER_OF_MODULES],
    swerve_constants_t *swerve_constants, float yaw, float previous_yaw, float dr);

#endif // SWERVE_ODOMETRY_H