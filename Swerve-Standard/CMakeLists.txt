cmake_minimum_required(VERSION 3.22)
project(Swerve-Standard C)

# Collect all source files specific to this robot
file(GLOB_RECURSE ROBOT_SOURCES "src/*.c")

# Add main.c from the typec-board-base
set(MAIN_SOURCE "${CMAKE_SOURCE_DIR}/control-base/typec-board-base/Core/Src/main.c")
set(CUSTOM_MAIN "${CMAKE_CURRENT_BINARY_DIR}/main_custom.c")

# Configure the custom main file with proper include path
configure_file(
    ${MAIN_SOURCE}
    ${CUSTOM_MAIN}
    COPYONLY
)

# Create a special include directory that will be used during build
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/include)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/inc/robot.h DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/include)

# Create the executable using the custom main
add_executable(Swerve-Standard 
    ${ROBOT_SOURCES} 
    ${CUSTOM_MAIN}
)

# Add include directories - put our special include first
target_include_directories(Swerve-Standard BEFORE PUBLIC
    ${CMAKE_CURRENT_BINARY_DIR}/include  # Special include directory with robot.h
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
    ${CMAKE_SOURCE_DIR}/control-base/algo/inc
    ${CMAKE_SOURCE_DIR}/control-base/bsp/inc
    ${CMAKE_SOURCE_DIR}/control-base/devices/inc
    ${CMAKE_SOURCE_DIR}/control-base/typec-board-base/Core/Inc
    ${CMAKE_SOURCE_DIR}/control-base/typec-board-base/Drivers/STM32F4xx_HAL_Driver/Inc
    ${CMAKE_SOURCE_DIR}/control-base/typec-board-base/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy
    ${CMAKE_SOURCE_DIR}/control-base/typec-board-base/Middlewares/Third_Party/FreeRTOS/Source/include
    ${CMAKE_SOURCE_DIR}/control-base/typec-board-base/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS
    ${CMAKE_SOURCE_DIR}/control-base/typec-board-base/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM4F
    ${CMAKE_SOURCE_DIR}/control-base/typec-board-base/Drivers/CMSIS/Device/ST/STM32F4xx/Include
    ${CMAKE_SOURCE_DIR}/control-base/typec-board-base/Drivers/CMSIS/Include
)

# Add compiler definitions
target_compile_definitions(Swerve-Standard PRIVATE
    USE_HAL_DRIVER
    STM32F407xx
    $<$<CONFIG:Debug>:DEBUG>
)

# Link against control_base library
target_link_libraries(Swerve-Standard PRIVATE control_base)

# Create additional build outputs (.bin and .hex files)
add_custom_command(TARGET Swerve-Standard POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary Swerve-Standard.elf Swerve-Standard.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex Swerve-Standard.elf Swerve-Standard.hex
    COMMENT "Creating Swerve-Standard.bin and Swerve-Standard.hex"
    VERBATIM
)

# Print the size of the executable
add_custom_command(TARGET Swerve-Standard POST_BUILD
    COMMAND ${CMAKE_SIZE} Swerve-Standard.elf
    COMMENT "Size of Swerve-Standard.elf:"
    VERBATIM
)