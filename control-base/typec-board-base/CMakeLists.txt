cmake_minimum_required(VERSION 3.22)

# Set the project name
set(CMAKE_PROJECT_NAME typec-board-base)

# Include toolchain file (Assumed to be correct for your target)
include(cmake/gcc-arm-none-eabi.cmake)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Add the STM32CubeMX library
add_subdirectory(cmake/stm32cubemx)

# Create the board base library (excluding main.c)
file(GLOB_RECURSE TYPEC_BOARD_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/Src/*.c"
)

# Explicitly exclude main.c since it needs robot-specific headers
list(FILTER TYPEC_BOARD_SOURCES EXCLUDE REGEX ".*main.c$")

add_library(typec-board-base STATIC
    ${TYPEC_BOARD_SOURCES}
)

target_include_directories(typec-board-base PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/Core/Inc"
    "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/STM32F4xx_HAL_Driver/Inc"
    "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Device/ST/STM32F4xx/Include"
    "${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include"
)

# Link with STM32CubeMX library
target_link_libraries(typec-board-base PRIVATE stm32cubemx)