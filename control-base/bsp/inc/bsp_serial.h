#ifndef BSP_SERIAL
#define BSP_SERIAL
#include <stdio.h>
#include "usart.h"

#define DEBUG

#ifdef DEBUG
#define DEBUG_PRINTF(huart, format, ...) debug_printf((huart), (format), ##__VA_ARGS__)
#else
#define DEBUG_PRINTF(huart, format, ...) ((void)0)
#endif

void debug_printf(UART_HandleTypeDef *huart, const char *format, ...);
extern int _write(int file, char *ptr, int len);
#endif
