cmake_minimum_required(VERSION 3.22)
project(Template)

# Collect sources
file(GLOB_RECURSE ROBOT_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c")
file(GLOB CONTROL_SOURCES "${CMAKE_SOURCE_DIR}/control-base/algo/src/*.c"
                         "${CMAKE_SOURCE_DIR}/control-base/bsp/src/*.c"
                         "${CMAKE_SOURCE_DIR}/control-base/devices/src/*.c")

# STM32 HAL sources
file(GLOB HAL_SOURCES "${CMAKE_SOURCE_DIR}/control-base/typec-board-base/Core/Src/*.c"
                     "${CMAKE_SOURCE_DIR}/control-base/typec-board-base/Drivers/STM32F4xx_HAL_Driver/Src/*.c")

# FreeRTOS sources (explicitly selected)
set(FREERTOS_SOURCES 
    "${CMAKE_SOURCE_DIR}/control-base/typec-board-base/Middlewares/Third_Party/FreeRTOS/Source/croutine.c"
    "${CMAKE_SOURCE_DIR}/control-base/typec-board-base/Middlewares/Third_Party/FreeRTOS/Source/event_groups.c"
    "${CMAKE_SOURCE_DIR}/control-base/typec-board-base/Middlewares/Third_Party/FreeRTOS/Source/list.c"
    "${CMAKE_SOURCE_DIR}/control-base/typec-board-base/Middlewares/Third_Party/FreeRTOS/Source/queue.c"
    "${CMAKE_SOURCE_DIR}/control-base/typec-board-base/Middlewares/Third_Party/FreeRTOS/Source/stream_buffer.c"
    "${CMAKE_SOURCE_DIR}/control-base/typec-board-base/Middlewares/Third_Party/FreeRTOS/Source/tasks.c"
    "${CMAKE_SOURCE_DIR}/control-base/typec-board-base/Middlewares/Third_Party/FreeRTOS/Source/timers.c"
    "${CMAKE_SOURCE_DIR}/control-base/typec-board-base/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS/cmsis_os.c"
    "${CMAKE_SOURCE_DIR}/control-base/typec-board-base/Middlewares/Third_Party/FreeRTOS/Source/portable/MemMang/heap_4.c"
    "${CMAKE_SOURCE_DIR}/control-base/typec-board-base/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c"
)

# ASM startup file
set(STARTUP_FILE "${CMAKE_SOURCE_DIR}/control-base/typec-board-base/startup_stm32f407xx.s")

# Remove main.c from HAL sources and file templates
list(FILTER HAL_SOURCES EXCLUDE REGEX "main\\.c$")
list(FILTER HAL_SOURCES EXCLUDE REGEX "template\\.c$")
list(FILTER HAL_SOURCES EXCLUDE REGEX "hal_timebase_rtc")

# Copy main.c and add custom include path
set(MAIN_FILE "${CMAKE_SOURCE_DIR}/control-base/typec-board-base/Core/Src/main.c")
set(CUSTOM_MAIN "${CMAKE_CURRENT_BINARY_DIR}/main_custom.c")
configure_file(${MAIN_FILE} ${CUSTOM_MAIN} COPYONLY)

# Add executable
add_executable(${PROJECT_NAME} 
    ${ROBOT_SOURCES}
    ${CONTROL_SOURCES}
    ${HAL_SOURCES}
    ${FREERTOS_SOURCES}
    ${STARTUP_FILE}
    ${CUSTOM_MAIN}
)

# Add robot includes
target_include_directories(${PROJECT_NAME} BEFORE PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
)

# Set specific options for main.c file
set_source_files_properties(${CUSTOM_MAIN} PROPERTIES COMPILE_FLAGS "-I${CMAKE_CURRENT_SOURCE_DIR}/inc")

# Create .hex and .bin files
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME} ${PROJECT_NAME}.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME} ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_SIZE} ${PROJECT_NAME}
)