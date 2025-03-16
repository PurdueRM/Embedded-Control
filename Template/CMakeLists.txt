cmake_minimum_required(VERSION 3.22)
project(Robot-Template)

# Find source files
file(GLOB_RECURSE TEMPLATE_SOURCES "src/*.c")

# Add main.c from the typec-board-base
set(ORIGINAL_MAIN "${CMAKE_SOURCE_DIR}/control-base/typec-board-base/Core/Src/main.c")
set(CUSTOM_MAIN "${CMAKE_CURRENT_BINARY_DIR}/main_custom.c")

# Configure the custom main file with proper include path
configure_file(
    ${ORIGINAL_MAIN}
    ${CUSTOM_MAIN}
    COPYONLY
)

# Create a special include directory that will be used during build
file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/include)
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/inc/robot.h DESTINATION ${CMAKE_CURRENT_BINARY_DIR}/include)

# Create the executable using the custom main
add_executable(${PROJECT_NAME} 
    ${TEMPLATE_SOURCES} 
    ${CUSTOM_MAIN}
)

# Add include directories - put our special include first
target_include_directories(${PROJECT_NAME} BEFORE PUBLIC
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
target_compile_definitions(${PROJECT_NAME} PRIVATE
    USE_HAL_DRIVER
    STM32F407xx
    $<$<CONFIG:Debug>:DEBUG>
)

# Link against control_base library
target_link_libraries(${PROJECT_NAME} PRIVATE control_base)

# Create additional build outputs (.bin and .hex files)
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary ${PROJECT_NAME}.elf ${PROJECT_NAME}.bin
    COMMAND ${CMAKE_OBJCOPY} -O ihex ${PROJECT_NAME}.elf ${PROJECT_NAME}.hex
    COMMENT "Creating ${PROJECT_NAME}.bin and ${PROJECT_NAME}.hex"
    VERBATIM
)

# Print the size of the executable
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_SIZE} ${PROJECT_NAME}.elf
    COMMENT "Size of ${PROJECT_NAME}.elf:"
    VERBATIM
)