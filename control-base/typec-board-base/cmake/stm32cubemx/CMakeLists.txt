cmake_minimum_required(VERSION 3.22)

project(stm32cubemx)
add_library(stm32cubemx INTERFACE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

target_compile_definitions(stm32cubemx INTERFACE 
	USE_HAL_DRIVER 
	STM32F407xx
    $<$<CONFIG:Debug>:DEBUG>
)

target_include_directories(stm32cubemx INTERFACE
    ../../Core/Inc
    ../../Drivers/STM32F4xx_HAL_Driver/Inc
    ../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    ../../Middlewares/Third_Party/FreeRTOS/Source/include
    ../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
    ../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    ../../Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ../../Drivers/CMSIS/Include
)

target_sources(stm32cubemx INTERFACE
    ../../Core/Src/main.c
    ../../Core/Src/gpio.c
    ../../Core/Src/freertos.c
    ../../Core/Src/can.c
    ../../Core/Src/dma.c
    ../../Core/Src/i2c.c
    ../../Core/Src/spi.c
    ../../Core/Src/tim.c
    ../../Core/Src/usart.c
    ../../Core/Src/stm32f4xx_it.c
    ../../Core/Src/stm32f4xx_hal_msp.c
    ../../Core/Src/stm32f4xx_hal_timebase_tim.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_i2c_ex.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c
    ../../Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c
    ../../Core/Src/system_stm32f4xx.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/croutine.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/event_groups.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/list.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/queue.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/tasks.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/timers.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c
    ../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
    ../../Core/Src/sysmem.c
    ../../Core/Src/syscalls.c
    ../../startup_stm32f407xx.s
)

target_link_directories(stm32cubemx INTERFACE
)

target_link_libraries(stm32cubemx INTERFACE
)

# Validate that STM32CubeMX code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()


