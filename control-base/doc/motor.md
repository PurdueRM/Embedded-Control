## Motor Initialization

an example usage
```C
Motor_Config_t yaw_motor_config = {
        // Comm Config
        .can_bus = 1, // set can bus currently using
        .speed_controller_id = 3,
        .offset = 3690,
        
        // Motor Reversal Config (if motor is installed in 
        // opposite direction, change to MOTOR_REVERSAL_REVERSED)
        .motor_reversal = MOTOR_REVERSAL_NORMAL,
        
        //external sensor config
        .use_external_feedback = 1,
        .external_feedback_dir = 1, // 1 if the feedback matches with task space direction, 0 otherwise
        .external_angle_feedback_ptr = &g_imu.rad.yaw, // assign the pointer to the external angle feedback
        .external_velocity_feedback_ptr = &(g_imu.bmi088_raw.gyro[2]), // assign the poitner to the external velocity feedback
        
        // Controller Config
        .control_mode = POSITION_CONTROL, // Control Mode, see control mode for detail
        .angle_pid =
            {
                .kp = 20000.0f,
                .kd = 1000000.0f,
                .output_limit = GM6020_MAX_CURRENT,
            },
        .velocity_pid =
            {
                .kp = 500.0f,
                .output_limit = GM6020_MAX_CURRENT,
            },
    };
```