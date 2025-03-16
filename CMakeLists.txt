cmake_minimum_required(VERSION 3.22)

# Set the toolchain
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/control-base/typec-board-base/cmake/gcc-arm-none-eabi.cmake")
set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)
set(CMAKE_ASM_COMPILER_WORKS 1)

# Define project
project(BoilerBots-Embedded C ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Linker script
set(LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/control-base/typec-board-base/STM32F407IGHx_FLASH.ld")

# Compiler and linker flags
add_compile_options(
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard
    -Wall
    -fdata-sections
    -ffunction-sections
)

set(CMAKE_EXE_LINKER_FLAGS "-T${LINKER_SCRIPT} -Wl,-Map=${CMAKE_BINARY_DIR}/output.map,--cref -Wl,--gc-sections -flto -u _printf_float")

# Common directories to include
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/control-base/algo/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/control-base/bsp/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/control-base/devices/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/control-base/typec-board-base/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/control-base/typec-board-base/Drivers/STM32F4xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/control-base/typec-board-base/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_SOURCE_DIR}/control-base/typec-board-base/Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/control-base/typec-board-base/Drivers/CMSIS/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/control-base/typec-board-base/Middlewares/Third_Party/FreeRTOS/Source/include
    ${CMAKE_CURRENT_SOURCE_DIR}/control-base/typec-board-base/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
    ${CMAKE_CURRENT_SOURCE_DIR}/control-base/typec-board-base/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
)

# Common defines
add_compile_definitions(
    USE_HAL_DRIVER
    STM32F407xx
)

# Add projects
add_subdirectory(Swerve-Standard)
add_subdirectory(Template)