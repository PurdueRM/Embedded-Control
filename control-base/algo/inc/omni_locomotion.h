#ifndef OMNI_LOCOMOTION_h
#define OMNI_LOCOMOTION_h

typedef struct
{
    float v_x;   // x velocity
    float v_y;   // y velocity
    float omega; // angular velocity

    float phi_dot_1; // wheel 1 angular velocity
    float phi_dot_2; // wheel 2 angular velocity
    float phi_dot_3; // wheel 3 angular velocity
    float phi_dot_4; // wheel 4 angular velocity
} omni_chassis_state_t;

// TODO. Copied from swerve odometry code
// Refactor this struct and other "duplicate" structs into
// a shared "types" file?
typedef struct 
{
    float x; // x position in meters
    float y; // y position in meters
    float theta; // orientation in radians
} pose_2d_t;

typedef struct
{
    float R;         // wheel diameter [m]
    float d;         // radius from center to wheel [m]
    float theta;     // mounting angle of omni wheel [rad]
    float max_speed; // maximum speed of the robot [m/s]

    pose_2d_t* init_pose;

    float kinematics_matrix[4][3]; // the matrix for kinematics
} omni_physical_constants_t;

// Assume motors start front_left and go counterclockwise
typedef struct
{
    float front_left;
    float back_left;
    float back_right;
    float front_right;
} motor_data_t;

// TODO. Change formatting to match official. Capitialize function words
omni_physical_constants_t omni_init(float R, float d, float theta, float max_speed, pose_2d_t* init_pose);
void omni_calculate_kinematics(omni_chassis_state_t *input, omni_physical_constants_t *omni_physical_constants);
void omni_desaturate_wheel_speeds(omni_chassis_state_t *input, omni_physical_constants_t *omni_physical_constants);
void omni_convert_to_rpm(omni_chassis_state_t *chassis_state);

void Init_Omni_Odometry(pose_2d_t* pose, pose_2d_t* init_pose, motor_data_t* curr_motor_data);
void Update_Omni_Odometry(pose_2d_t* pose, omni_physical_constants_t* omni_physical_constants, motor_data_t* curr_motor_data, float heading_angle);

#endif // OMNI_LOCOMOTION_h