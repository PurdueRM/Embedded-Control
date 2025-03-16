cmake_minimum_required(VERSION 3.22)

project(stm32cubemx)
add_library(stm32cubemx STATIC)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Add specific HAL modules that need to be enabled
target_compile_definitions(stm32cubemx PRIVATE 
    USE_HAL_DRIVER 
    STM32F407xx
    HAL_CAN_MODULE_ENABLED  # Ensure CAN module is enabled
    $<$<CONFIG:Debug>:DEBUG>
)

# Add STM32CubeMX generated sources
file(GLOB CORE_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Src/*.c"
)

# Add HAL Driver sources
file(GLOB_RECURSE HAL_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Src/*.c"
)

# Add only the specific FreeRTOS files we need (like in the original Makefile)
set(FREERTOS_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/croutine.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/event_groups.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/list.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/queue.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/tasks.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/timers.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c"
)

# Combine all sources
set(CUBEMX_SOURCES ${CORE_SOURCES} ${HAL_SOURCES} ${FREERTOS_SOURCES})

# Exclude main.c from compilation
list(FILTER CUBEMX_SOURCES EXCLUDE REGEX ".*main.c$")

# Exclude template files from compilation
list(FILTER CUBEMX_SOURCES EXCLUDE REGEX ".*_template\\.c$")

# Exclude RTC alarm timebase specifically
list(FILTER CUBEMX_SOURCES EXCLUDE REGEX ".*hal_timebase_rtc.*\\.c$")

# Add sources to library
target_sources(stm32cubemx PRIVATE ${CUBEMX_SOURCES})

# Use absolute paths to ensure correct inclusion
set(CONTROL_BASE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../../control-base")

# Also create a forced include file to set HAL_CAN_MODULE_ENABLED
file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/stm32_hal_conf_extra.h 
"#ifndef STM32_HAL_CONF_EXTRA_H
#define STM32_HAL_CONF_EXTRA_H

/* Ensure CAN module is enabled */
#ifndef HAL_CAN_MODULE_ENABLED
#define HAL_CAN_MODULE_ENABLED
#endif

#endif /* STM32_HAL_CONF_EXTRA_H */
")

# Include directories for STM32CubeMX
target_include_directories(stm32cubemx PUBLIC
    ${CMAKE_CURRENT_BINARY_DIR}  # For stm32_hal_conf_extra.h
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Drivers/CMSIS/Include
    # Use absolute paths for more reliability
    "${CONTROL_BASE_DIR}/algo/inc"
    "${CONTROL_BASE_DIR}/bsp/inc"
    "${CONTROL_BASE_DIR}/devices/inc"
)

# Add a forced include directive for our extra header
target_compile_options(stm32cubemx PRIVATE
    -include ${CMAKE_CURRENT_BINARY_DIR}/stm32_hal_conf_extra.h
)

# Add compiler options for floating point hardware
target_compile_options(stm32cubemx PRIVATE
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
)

# Validate that STM32CubeMX code is compatible with C standard
if(CMAKE_C_STANDARD LESS 11)
    message(ERROR "Generated code requires C11 or higher")
endif()


