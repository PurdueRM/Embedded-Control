#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

#define SPIN_TOP_OMEGA 6 // rad / s
#define MAX_ANGLUAR_SPEED 3.14 // rad / s
#define CHASSIS_WHEEL_DIAMETER (0.15f) // m
#define CHASSIS_RADIUS (0.21f) // center to wheel, m
#define CHASSIS_MAX_SPEED (2.0f) // m/s
#define CHASSIS_MOUNTING_ANGLE (PI / 4) // rad (45deg)
#define MAX_ABC (200.0f) // rad/s // TODO rename this macro

// Function prototypes
void Chassis_Task_Init(void);
void Chassis_Ctrl_Loop(void);

#endif // CHASSIS_TASK_H
