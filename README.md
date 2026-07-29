![Project Logo](images/logo.png)


# WARNING, this is a temporary README

If you need to set up this repo, refer to the [old control-base repo's README](https://github.com/PurdueRM/control-base)


Ports on h7 board

Can:
- CAN1 (PD00 RX, PD01 TX)
- CAN2 (PB05 RX, PB06 TX)
- CAN3 (PD12 RX, PD13 TX)

Uart:
- USART1 -- Asynchronous (PA10 RX, PA09 TX)
- UART5 (RX Only) -- SBUS (PD02 RX)
- UART7 (PE07 RX, PE08 TX)
- UART10 (PE02 RX, PE03 TX)

RS485:
- USART2 -- Half-duplex (PD06 RX, PD05 TX, PD04 DE)
- USART3 -- Half-duplex (PD09 RX, PD08 TX, PB14 DE)

USB (PA11 DM, PA12 DP)

Misc:
- BUZZER (PB15 TIM12_CH2)
- IMU (SPI2)
    - PB13 SCK, PC01 MOSI, PC02 MISO, PC00 CS0, PC03 CS1
    - PE10 INT1, PE12 INT3, PB04 TM3_CH4
- SWD (PA13 SWDIO, PA14 SWCLK)
- LED (PA07 SPI6_MOSI) --> not enabled in cubemx
- LCD (SPI1 + I2C2)
    - PB03 SCK, PD07 MOSI, PB04 MISO, PE15 CS
    - PB10 SCL, PB11 SDA
    - PD10 GPIO, PA05 ADC1_CH18
- DCMI Camera Interface (Not Configuring)
- 4x PWM
    - PE13 TIM1_CH3
    - PE09 TIM1_CH1
    - PA02 TIM2_CH3
    - PA00 TIM2_CH1

There is an expansion port which gives access to:
- UART9
- UART8
- SPI3
- I2C

