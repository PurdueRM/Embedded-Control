#ifndef CHASSIS_TASK_H
#define CHASSIS_TASK_H

// PHYSICAL CONSTANTS
#define SWERVE_MAX_SPEED 1.4f          // m/s
#define SPIN_TOP_OMEGA 4.0f            // m/s
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
// Uses up as much of the wattage as possible
// Has ~5 watt charging buffer
// So W100, robot uses 95 watts on average
#define SPINTOP_OMEGA_W45   4.9f
#define SPINTOP_OMEGA_W50   5.4f
#define SPINTOP_OMEGA_W55   6.05f
#define SPINTOP_OMEGA_W60   6.6f
#define SPINTOP_OMEGA_W65   7.1f
#define SPINTOP_OMEGA_W70   7.6f
#define SPINTOP_OMEGA_W75   8.1f
#define SPINTOP_OMEGA_W80   8.55f
#define SPINTOP_OMEGA_W90   9.4f
#define SPINTOP_OMEGA_W100  9.9f

// Function prototypes
void Chassis_Task_Init(void);
void Chassis_Ctrl_Loop(void);
float Rescale_Chassis_Velocity(void);
void Update_Maxes(void);

#endif // CHASSIS_TASK_H
