#include "rate_limiter.h"

/**
 * @brief Initialize the rate limiter
 * 
 * @param rate_limiter The rate limiter struct
 * @param rate_limit The rate limit in units per second
 */
void rate_limiter_init(rate_limiter_t *rate_limiter, float rate_limit)
{
    rate_limiter->rate_limit = rate_limit;
    rate_limiter->prev_value = 0;
    rate_limiter->prev_time = xTaskGetTickCount();
}

/**
 * @brief Limit the rate of change of a value
 * 
 * @param rate_limiter The rate limiter struct
 * @param input The input value
 * @return float The limited value
 */
float rate_limiter(rate_limiter_t *rate_limiter, float input)
{
    uint32_t curr_time = xTaskGetTickCount();
    float dt = (curr_time - rate_limiter->prev_time) / (float)configTICK_RATE_HZ; // convert to seconds
    float max_delta = rate_limiter->rate_limit * dt;

    float delta = input - rate_limiter->prev_value;
    if (fabsf(delta) > max_delta)
    {
        input = rate_limiter->prev_value + (delta > 0 ? max_delta : -max_delta);
    }

    rate_limiter->prev_value = input;
    rate_limiter->prev_time = curr_time;

    return input;
}