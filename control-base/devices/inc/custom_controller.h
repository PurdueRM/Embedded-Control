#ifndef CUSTOM_CONTROLLER_H
#define CUSTOM_CONTROLLER_H

#include <stdint.h>

typedef struct {
    uint8_t reset_heading;
    uint8_t teleop_toggle;
    uint8_t ee_trigger;
    uint8_t ee_twist;
} controller_flags_t;

typedef struct{
    uint8_t seq_num;
    float angle[3];    // [0]=X, [1]=Y, [2]=Z
    float acc[3];      // [0]=X, [1]=Y, [2]=Z
    float ang_vel[3];  // [0]=X, [1]=Y, [2]=Z
    uint8_t joystick_x;
    uint8_t joystick_y;
    controller_flags_t flags;
} custom_controller_t;

#endif