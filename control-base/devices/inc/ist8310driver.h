#ifndef IST8310DRIVER_H
#define IST8310DRIVER_H

#include <stdint.h>

#define IST8310_DATA_READY_BIT 2

#define IST8310_NO_ERROR 0x00

#define IST8310_NO_SENSOR 0x40

typedef struct IST8310_Raw_t
{
  uint8_t status;
  float mag[3];
} IST8310_Raw_t;

extern uint8_t ist8310_init(void);
extern void ist8310_read_over(uint8_t *status_buf, IST8310_Raw_t *mpu6500_real_data);
extern void ist8310_read_mag(float mag[3]);
#endif
