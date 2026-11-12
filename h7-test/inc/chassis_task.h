#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#define SIN45 0.70710678
#define COS45 0.70710678
#define WHEEL_RADIUS 2.0f
#define WHEEL_DIST 2.0f
#define MAX_SPEED 600.0f

struct Wheel_Velos {
    float wheel_one_speed;
    float wheel_two_speed;
    float wheel_three_speed;
    float wheel_four_speed;
};

struct Input {
    float x_velo;
    float y_velo;
    float ang_velo;
};

// Function prototypes
void Chassis_Task_Init(void);
void Chassis_Ctrl_Loop(void);

#endif // CHASSIS_TASK_H
