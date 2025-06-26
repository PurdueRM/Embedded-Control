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

#define OMEGA_55W 5.3f
#define OMEGA_65W 5.9f
#define OMEGA_70W 6.7f
#define OMEGA_75W 7.0f
#define OMEGA_80W 7.3f
#define OMEGA_85W 7.5f
#define OMEGA_90W 7.5f

// Function prototypes
void Chassis_Task_Init(void);
void Chassis_Ctrl_Loop(void);
void Update_Omega();

#endif // CHASSIS_TASK_H
