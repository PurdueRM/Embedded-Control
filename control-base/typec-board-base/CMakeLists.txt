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

# Create a LIBRARY object type
add_library(${CMAKE_PROJECT_NAME} STATIC)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup (Usually not needed for libraries)
# target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE ...)

# Add sources to library
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    startup_stm32f407xx.s
    STM32F407IGHx_FLASH.ld
    # Add other user sources from typec-board-base here, if any
)

# Add include paths (If you have any headers specific to typec-board-base)
# target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE ...)

# Add project symbols (macros)
# target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE ...)

# Link the generated stm32cubemx library.  This assumes stm32cubemx
# creates a library named 'stm32cubemx'.
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE stm32cubemx)

# No need to set C_STANDARD here; it's set globally.