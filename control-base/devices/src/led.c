#include "led.h"

#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "spi.h"

void LED_Start() {
    float brightPercent = .05f; // Percent from 1.0 to 0.0
    uint8_t maxColorValue = 255.0f; // Max RGB color value
    uint8_t scaledColorValue = (float)maxColorValue * brightPercent;

    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t TimeIncrement = pdMS_TO_TICKS(15);

    uint8_t r = 0;
    uint8_t g = 0;
    uint8_t b = 0;

    uint16_t step = 0;
    const uint16_t steps_per_segment = 50;   
    const uint16_t steps_per_cycle = steps_per_segment * 6;

    while (1)
    {
        // set tx buffer
        uint8_t txbuf[24];  // 8 bytes per color channel
        for (int bit = 0; bit < 8; bit++)
        {
            txbuf[7  - bit] = (((g >> bit) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
            txbuf[15 - bit] = (((r >> bit) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
            txbuf[23 - bit] = (((b >> bit) & 0x01) ? WS2812_HighLevel : WS2812_LowLevel) >> 1;
        }

        // transmit colors
        (void)HAL_SPI_Transmit(&hspi6, txbuf, sizeof(txbuf), 0xFFFF);

        // reset pulse (has to cover 50 microseconds)
        // uint8_t zero = 0x00; 
        // while (hspi6.State != HAL_SPI_STATE_READY) {
        //     (void)HAL_SPI_Transmit(&hspi6, &zero, 1, 20);
        // }

        // update colors
        uint16_t s = step % steps_per_cycle;
        int segment = s / steps_per_segment;
        float pos = (float)(s % steps_per_segment) / (float)steps_per_segment;

        switch (segment) {
            case 0: // Red → Yellow
                r = scaledColorValue;
                g = (uint8_t)((float)scaledColorValue * pos);
                b = 0;
                break;
            case 1: // Yellow → Green
                r = (uint8_t)((float)scaledColorValue* (1.0f - pos));
                g = scaledColorValue;
                b = 0;
                break;
            case 2: // Green → Cyan
                r = 0;
                g = scaledColorValue;
                b = (uint8_t)((float)scaledColorValue * pos);
                break;
            case 3: // Cyan → Blue
                r = 0;
                g = (uint8_t)((float)scaledColorValue * (1.0f - pos));
                b = scaledColorValue;
                break;
            case 4: // Blue → Magenta
                r = (uint8_t)((float)scaledColorValue * pos);
                g = 0;
                b = scaledColorValue;
                break;
            default: // 5: Magenta → Red
                r = scaledColorValue;
                g = 0;
                b = (uint8_t)((float)scaledColorValue * (1.0f - pos));
                break;
        }

        step++;

        vTaskDelayUntil(&xLastWakeTime, TimeIncrement);
    }
}
