#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#define SPIN_TOP_OMEGA 6 // rad / s
#define MAX_ANGLUAR_SPEED 6.28 // rad / s
#define CHASSIS_WHEEL_DIAMETER (0.15f) // m
#define CHASSIS_RADIUS (0.29f) // center to wheel, m
#define CHASSIS_MAX_SPEED (2.0f) // m/s
#ifndef PI
#define PI (3.14159265358979323846f)
#endif // PI

#define CHASSIS_MOUNTING_ANGLE (PI / 4) // rad (45deg)
#define MAX_ABC (400.0f) // rad/s // TODO rename this macro

#define INIT_X_POS (0.0f) // Starting 2d x position
#define INIT_Y_POS (0.0f) // Starting 2d y position
#define INIT_THETA (0.0f) // Starting Heading

// Function prototypes
void Chassis_Task_Init(void);
void Chassis_Ctrl_Loop(void);

#endif // CHASSIS_TASK_H
