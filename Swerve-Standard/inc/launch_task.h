#ifndef LAUNCH_TASK_H
#define LAUNCH_TASK_H

#include "dji_motor.h"

#define NUM_SHOTS 8
#define SHOT_ANGLE_OFFSET_RAD (2 * PI / NUM_SHOTS)
#define FEED_TOLERANCE (5 * PI / 180) // 5 degree tolerance 
#define FEED_RATE  60 / 8 * 60 // 60 HZ

void Launch_Task_Init(void);
void Launch_Ctrl_Loop(void);
void handleSingleFire(void);
void startFlywheel(void);
void stopFlywheel(void);
void handleFullAuto(void);

/**
 * @brief Rejiggle the feed motor to prevent jams
 */
void rejiggle(void);

#endif // LAUNCH_TASK_H
