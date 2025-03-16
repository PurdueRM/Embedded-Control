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

# Create the control_base library
add_library(control_base STATIC
    ${CONTROL_BASE_SOURCES}
)

# Include directories
target_include_directories(control_base PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/algo/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/inc
)

# Link with typec-board-base
target_link_libraries(control_base PUBLIC typec-board-base)

# Export include directories to dependent targets
set(CONTROL_BASE_INCLUDE_DIRS 
    ${CMAKE_CURRENT_SOURCE_DIR}/algo/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/bsp/inc
    ${CMAKE_CURRENT_SOURCE_DIR}/devices/inc
    CACHE INTERNAL "Control base include directories"
)