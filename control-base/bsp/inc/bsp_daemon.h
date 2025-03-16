#ifndef BSP_DAEMON_H
#define BSP_DAEMON_H

#include <stdint.h>

#define DAEMON_PERIOD (10) // unit: ms

typedef struct
{
    uint8_t id;
    uint16_t counter;
    uint16_t reload_value;

    void (*callback)(void);
} Daemon_Instance_t;

void Daemon_Init(void);
Daemon_Instance_t* Daemon_Register(uint16_t reload_value, uint16_t initial_counter, void (*callback)(void));
void Daemon_Reload(Daemon_Instance_t *daemon);
void Daemon_Task_Loop(void);

#endif // BSP_DAEMON_H