#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

// PHYSICAL CONSTANTS
#define SWERVE_MAX_SPEED 1.0f          // m/s
#define SPIN_TOP_OMEGA 2.0f            // m/s
#define SWERVE_MAX_ANGLUAR_SPEED 3.14f // rad/s
#define TRACK_WIDTH 0.34f              // m, measured wheel to wheel (side to side)
#define WHEEL_BASE 0.34f               // m, measured wheel to wheel (up and down)
#define WHEEL_DIAMETER 0.12f           // m, measured wheel diameter
#define SWERVE_MAX_WHEEL_ACCEL 1.0f           // m/s^2

// Function prototypes
void Chassis_Task_Init(void);
void Chassis_Ctrl_Loop(void);
float Rescale_Chassis_Velocity(void);

#endif // CHASSIS_TASK_H
