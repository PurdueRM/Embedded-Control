cmake_minimum_required(VERSION 3.22)

# Set the toolchain file BEFORE project() command
set(CMAKE_TOOLCHAIN_FILE "${CMAKE_CURRENT_SOURCE_DIR}/control-base/typec-board-base/cmake/gcc-arm-none-eabi.cmake" CACHE FILEPATH "Toolchain file")

# Prevent CMake from testing the compiler
set(CMAKE_C_COMPILER_WORKS 1)
set(CMAKE_CXX_COMPILER_WORKS 1)
set(CMAKE_ASM_COMPILER_WORKS 1)

# Define the project after toolchain file
project(BoilerBots-Embedded LANGUAGES C ASM)

# Set global properties
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Build the control base library first
add_subdirectory(control-base)

# Add your executable projects
add_subdirectory(Swerve-Standard)
# Add other projects as needed
# add_subdirectory(Template)