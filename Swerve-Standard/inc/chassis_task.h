#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

// PHYSICAL CONSTANTS
#define SWERVE_MAX_SPEED 1.4f          // m/s
#define SPIN_TOP_OMEGA 3.5f            // m/s
#define SWERVE_MAX_ANGLUAR_SPEED 3.14f // rad/s
#define TRACK_WIDTH 0.34f              // m, measured wheel to wheel (side to side)
#define WHEEL_BASE 0.34f               // m, measured wheel to wheel (up and down)
#define WHEEL_DIAMETER 0.12f           // m, measured wheel diameter
#define SWERVE_MAX_WHEEL_ACCEL 0.8f    // m/s^2
#define SWERVE_QUICK_STOP_ACCEL 3.0f   // m/s^2

// Max Speeds 
#define MAX_SPEED_W45   1.5f
#define MAX_SPEED_W50   1.5f
#define MAX_SPEED_W55   1.5f
#define MAX_SPEED_W60   1.5f
#define MAX_SPEED_W65   1.5f
#define MAX_SPEED_W70   1.5f
#define MAX_SPEED_W75   1.5f
#define MAX_SPEED_W80   1.5f
#define MAX_SPEED_W90   1.5f
#define MAX_SPEED_W100  1.5f

// Spintop Omegas 
#define SPINTOP_OMEGA_W45   4.0f
#define SPINTOP_OMEGA_W50   4.5f
#define SPINTOP_OMEGA_W55   5.0f
#define SPINTOP_OMEGA_W60   5.5f
#define SPINTOP_OMEGA_W65   6.0f
#define SPINTOP_OMEGA_W70   6.5f
#define SPINTOP_OMEGA_W75   7.0f
#define SPINTOP_OMEGA_W80   7.5f
#define SPINTOP_OMEGA_W90   8.0f
#define SPINTOP_OMEGA_W100  8.5f

// Function prototypes
void Chassis_Task_Init(void);
void Chassis_Ctrl_Loop(void);
float Rescale_Chassis_Velocity(void);
void Update_Maxes(void);

#endif // CHASSIS_TASK_H
