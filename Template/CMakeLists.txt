cmake_minimum_required(VERSION 3.22)
project(Robot-Template)

# Find source files
file(GLOB_RECURSE TEMPLATE_SOURCES "src/*.c")

# Create the executable
add_executable(${PROJECT_NAME} ${TEMPLATE_SOURCES})

# Add include directories
target_include_directories(${PROJECT_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/inc
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