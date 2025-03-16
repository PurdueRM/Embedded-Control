# control-base/CMakeLists.txt
cmake_minimum_required(VERSION 3.22)

# Do NOT redefine the project languages as it may reset compiler settings
project(control_base)

# Build typec-board-base first
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/typec-board-base)

# Find source files in control-base directory (adjust paths as needed)
file(GLOB_RECURSE CONTROL_BASE_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/algo/src/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/bsp/src/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/devices/src/*.c"
)

# Exclude main.c as it needs robot-specific headers
list(FILTER CONTROL_BASE_SOURCES EXCLUDE REGEX ".*typec-board-base/Core/Src/main.c$")

# Create the control_base library
add_library(control_base STATIC
    ${CONTROL_BASE_SOURCES}
)

# Define include directories
set(CONTROL_BASE_INCLUDE_DIRS 
    ${CMAKE_CURRENT_SOURCE_DIR}/algo/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/typec-board-base/Core/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/typec-board-base/Drivers/STM32F4xx_HAL_Driver/Inc
    ${CMAKE_CURRENT_SOURCE_DIR}/typec-board-base/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    ${CMAKE_CURRENT_SOURCE_DIR}/typec-board-base/Middlewares/Third_Party/FreeRTOS/Source/include
    ${CMAKE_CURRENT_SOURCE_DIR}/typec-board-base/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
    ${CMAKE_CURRENT_SOURCE_DIR}/typec-board-base/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    ${CMAKE_CURRENT_SOURCE_DIR}/typec-board-base/Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/typec-board-base/Drivers/CMSIS/Include
)

# Include directories
target_include_directories(control_base PUBLIC
    ${CONTROL_BASE_INCLUDE_DIRS}
)

# Add compiler definitions
target_compile_definitions(control_base PUBLIC
    USE_HAL_DRIVER
    STM32F407xx
)

# Link with typec-board-base
target_link_libraries(control_base PUBLIC typec-board-base)