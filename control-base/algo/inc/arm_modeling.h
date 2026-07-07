#ifndef ARM_MODELING_H
#define ARM_MODELING_H

#include <stdio.h>
#include <math.h>

#include "user_math.h"

/*
l1: length of first arm
l2: length of second arm
h1: height from base of shoulder motor
h2: offset of second arm (it is L shaped)
h3: offset of spherical wrist center
h4: height of motor (dist from center of spherical wrist to end effector)
*/
#define L1 0.37f    // Shoulder-elbow
#define L2 0.167f   // Elbow-wrist
#define H1 0.10f    // Base height (Assuming 0.1m, adjust if needed)
#define H2 0.167f   // Wrist-finger
#define H3 0.09f    // Finger-knuckle
#define H4 0.295f   // Knuckle-EE (0.095m) + EE Length (0.2m)

#define DAMPING 0.01f
#define EPSILON 0.000001f

typedef struct joint_t {
    DH_Params dh_params;
    float theta;
    float velocity;
    float x_pos;
    float y_pos;
    float z_pos;
} joint_t;

typedef struct Arm_State_t {
    joint_t joints[6];
    joint_t sim_joints[6];
    Mat *theta_lims;

    Mat *jacobian;
    Mat *pseudo_inv_jacobian;
    Mat *nullspace;
    Mat *transforms[7];
    Mat *sim_transforms[7];

    Mat *desired_orientation_mat;
    Mat *current_orientation_mat;
    Vec *orientation_error;

    Vec *wrist_pos;
    Vec *wrist_pos_desired;

    Vec *desired_pos;

    Vec *theta;
    // the buffers exist to avoid malloc/free in the control loop
    
    Mat *buffer1_6x6;
    Mat *buffer2_6x6;

    Mat *buffer1_6x5;

    Mat *buffer1_5x5;
    Mat *buffer2_5x5;
    Mat *buffer3_5x5;

    Mat *buffer1_4x4;
    Mat *buffer2_4x4;

    Mat *buffer1_3x3;
    Mat *buffer2_3x3;
    Mat *buffer3_3x3;

    Vec *buffer1_3x1;
    Vec *buffer2_3x1;

    Vec *buffer1_5x1;

    Vec *buffer1_6x1;
    Vec *buffer2_6x1;
    Vec *buffer3_6x1;

    
} Arm_State_t;

Arm_State_t g_arm_state;

void init_arm_state(Arm_State_t *state);

void inverse_kinematics(float* theta, Vec* target_pos, Mat* target_rot, int max_iters, float tol, float damping);

#endif
