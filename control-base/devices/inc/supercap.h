#ifndef __SUPERCAP_H
#define __SUPERCAP_H

#include <stdint.h>
#include "bsp_can.h"
#include "referee_system.h"
#include "jetson_orin.h"

#define SUPERCAP_POWER (20)

typedef struct
{
    uint8_t can_bus;
    uint16_t tx_id;
    uint16_t rx_id;
    uint8_t send_counter;

    uint8_t supercap_percent;
    uint8_t supercap_enabled_flag;
} Supercap_t;

void Supercap_Init(Supercap_t *g_supercap);
void Supercap_Decode(CAN_Instance_t *can_instance);
void Supercap_Send(void);

extern Supercap_t g_supercap;

#endif // __SUPERCAP_H