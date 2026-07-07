#ifndef GIMBAL_TASK_H
#define GIMBAL_TASK_H

#define YAW_MID_POSITION
#define PITCH_MID_POSITION

typedef struct
{
    float pitch;
    float yaw_velocity;
    float yaw_angle;
} Gimbal_Target_t;

// Function prototypes
void Gimbal_Task_Init(void);
void Gimbal_Ctrl_Loop(void);

#endif // GIMBAL_TASK_H
