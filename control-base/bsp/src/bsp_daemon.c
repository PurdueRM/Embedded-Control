#include "bsp_daemon.h"
#include <stdlib.h>

#define DAEMON_INSTANCE_MAX (3)
Daemon_Instance_t *g_daemon_instances[DAEMON_INSTANCE_MAX];
uint8_t g_daemon_instance_count = 0;

Daemon_Instance_t *Daemon_Register(uint16_t reload_value, uint16_t initial_counter, void (*callback)(void))
{
    Daemon_Instance_t *daemon = (Daemon_Instance_t *)malloc(sizeof(Daemon_Instance_t));

    daemon->counter = initial_counter;
    daemon->reload_value = reload_value;
    daemon->callback = callback;

    g_daemon_instances[g_daemon_instance_count++] = daemon;
    return daemon;
}

void Daemon_Reload(Daemon_Instance_t *daemon)
{
    daemon->counter = daemon->reload_value;
}

void Daemon_Task_Loop()
{
    for (int i = 0; i < g_daemon_instance_count; i++)
    {
        g_daemon_instances[i]->counter--;
        if (g_daemon_instances[i]->counter == 0)
        {
            g_daemon_instances[i]->callback();
            g_daemon_instances[i]->counter = g_daemon_instances[i]->reload_value;
        }
    }
}