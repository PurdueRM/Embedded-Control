#ifndef RATE_LIMITER_H
#define RATE_LIMITER_H

#include <math.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"

typedef struct {
    float rate_limit; // !units per second
    float prev_value;
    float prev_time;
} rate_limiter_t;

void rate_limiter_init(rate_limiter_t *rate_limiter, float rate_limit);
float rate_limiter(rate_limiter_t *rate_limiter, float input);

#endif // RATE_LIMITER_H