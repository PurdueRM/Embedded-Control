#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#define YAW_MID_POSITION
#define PITCH_MID_POSITION

#define YAW_OFFSET (0.0f)

typedef struct
{
    float pitch;
    float yaw_velocity;
    float yaw_angle;
} Gimbal_Target_t;

// Function prototypes
void Gimbal_Task_Init(void);
void Gimbal_Ctrl_Loop(void);
void Gimbal_Task_Disable(void);
#endif // GIMBAL_TASK_H