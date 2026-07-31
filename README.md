# Purdue Robomasters Control Base

This repository contains the shared control code between Purdue RoboMaster robots.

## Repository Initialization Guide

```bash
git clone [https://github.com/RoboMaster-Club/control-base.git](https://github.com/RoboMaster-Club/control-base.git)
cd control-base
git submodule update --init

Markdown

# Purdue Robomasters Control Base

This repository contains the shared control code between Purdue RoboMaster robots.

## Repository Initialization Guide

```bash
git clone [https://github.com/RoboMaster-Club/control-base.git](https://github.com/RoboMaster-Club/control-base.git)
cd control-base
git submodule update --init

VSCode Makefile Environment Setup Guide
Install Tools

VSCode: Download VSCode from here.

Package Manager:

    Windows: Install WSL (Ubuntu recommended) and the WSL Extension in VSCode.

    MacOS: Install homebrew.

    Linux: Use the one that comes with your system (e.g., apt).

Arm GNU Tools and OpenOCD:

    Windows: Open your WSL terminal (e.g., Ubuntu) and follow the Linux installation instructions below.

    MacOS: Run these commands in your terminal:

Bash

brew install gcc-arm-embedded
brew install openocd

    Linux / WSL: Run these commands in your Linux/WSL terminal:

Bash

sudo apt update
sudo apt upgrade
sudo apt install openocd gcc-arm-none-eabi gdb-multiarch

Check installation and environment variables:

    Check tool path installations using which in your Linux/WSL or MacOS terminal (e.g., which openocd or which gdb-multiarch). Remember the path for the GDB binary, as this will be used to configure the debug extension.

Set Up VSCode

Install VSCode extensions:

    Install the VSCode extension Cortex-Debug to enable ARM microcontroller debugging.

    If using Windows, connect VSCode to your WSL instance by running code . inside your WSL terminal directory or clicking the Remote Window icon in the bottom-left corner of VSCode.

Modify Extension settings: Add the GDB path by opening your VSCode settings.json and adding the following entry. Use the path found using which gdb-multiarch (or which arm-none-eabi-gdb on MacOS).

    Windows (WSL) / Linux: "cortex-debug.gdbPath": "/usr/bin/gdb-multiarch"

    MacOS: "cortex-debug.gdbPath": "/opt/homebrew/bin/arm-none-eabi-gdb"

[Optional] VSCode IntelliSense Configuration: Adding this to c_cpp_properties.json will link the standard library header files (e.g., stdint.h, stdlib.h, math.h).

    Windows (WSL) / Linux:
    JSON

    "C_Cpp.default.compilerPath": "/usr/bin/arm-none-eabi-gcc"

    MacOS:
    JSON

    "C_Cpp.default.compilerPath": "/opt/homebrew/bin/arm-none-eabi-gcc"

Usage
Tasks

VSCode tasks are used to do things like build, clean, or flash the project. Task configurations are located in tasks.json.

    Open the Command Palette in VSCode using [Ctrl/Cmd+Shift+P].

    Select Tasks: Run Task and pick the appropriate task.

    You can use the shortcut [Ctrl/Cmd+Shift+B] to run the default build task.

Debugging

A debug session can be used to test and diagnose issues. Debug configurations are located in launch.json.

    Navigate to Run and Debug in VSCode or use [Ctrl/Cmd+Shift+D].

    Select the appropriate launch configuration depending on whether you are using ST-LINK or CMSIS-DAP debugger.

    Click on the green play button or press [F5] to start a debug session.

    Note for Windows (WSL) users: USB debuggers attached to Windows must be passed through to WSL using usbipd-win.

Motor Config Example
Code snippet

Motor_Config_t yaw_motor_config = {
        // Comm Config
        .can_bus = 1, // set can bus currently using
        .speed_controller_id = 3,
        .offset = 3690,

        // Motor Reversal Config (if motor is installed in
        // opposite direction, change to MOTOR_REVERSAL_REVERSED)
        .motor_reversal = MOTOR_REVERSAL_NORMAL,

        // External sensor config
        .use_external_feedback = 1,
        .external_feedback_dir = 1, // 1 if feedback matches task space direction, 0 otherwise
        .external_angle_feedback_ptr = &g_imu.rad.yaw, // assign pointer to external angle feedback
        .external_velocity_feedback_ptr = &(g_imu.bmi088_raw.gyro[2]), // assign pointer to external velocity feedback

        // Controller Config
        .control_mode = POSITION_CONTROL, // Control Mode, see control mode for details
        .angle_pid =
            {
                .kp = 20000.0f,
                .kd = 1000000.0f,
                .output_limit = GM6020_MAX_CURRENT,
            },
        .velocity_pid =
            {
                .kp = 500.0f,
                .output_limit = GM6020_MAX_CURRENT,
            },
    };

Development Conventions

Code Formatting:
C

// All names must use snake_case.

// Variable names are all lowercase.
float example_float = 1.5f;

// Macros should be all UPPERCASE, and enclosed by ().
#define EXAMPLE_MACRO (3.14f)

// Function names should capitalize the first letter of each word.
float Example_Function() {}

// typedef names should capitalize the first letter of each word and end in _t.
typedef struct _Example_Struct_s Example_Typedef_t {}

// Enum names should capitalize the first letter of each word and end in _e.
enum Example_Enum_e {};

/* In general, indent code blocks for functions and if statements as such,
but for switches put cases in the same line. */
void Example_Func()
{
   if (some_condition)
   {
      switch(some_num)
      {
      case 0:
         break;
      default:
         break;
      }
   }
}

// For multiline macros, indent as such:
#define YOUR_MACRO        \
   {                      \
        FIRST_LINE = 0,   \
        SECOND_LINE = 1,  \
        THIRD_LINE = 2,   \
   }

Common Issues
1. Windows / WSL fails to initialize or see USB debuggers (CMSIS-DAP, ST-LINK).

Solution:

    Install usbipd-win on Windows (winget install --interactive --exact dorssel.usbipd-win).

    Open PowerShell as Administrator on Windows and list connected devices:
    PowerShell

    usbipd list

    Bind and attach the debugger to WSL (replace <busid> with your device's ID):
    PowerShell

    usbipd bind --busid <busid>
    usbipd attach --wsl --busid <busid>

2. Tools (openocd, make, arm-none-eabi-gcc) not found

Failed to launch OpenOCD GDB Server:...

or

make: command not found

Solution:
Ensure all necessary packages are installed in your WSL/Linux environment:
Bash

sudo apt update
sudo apt install build-essential openocd gcc-arm-none-eabi gdb-multiarch

Verify tool locations using which openocd or which arm-none-eabi-gcc. If custom paths are needed, configure "serverpath" in .vscode/launch.json or update your PATH variable in ~/.bashrc.
Standard Debug Procedure

These are common errors to check for:

    Ensure the IMU is firmly attached.

    Verify remote functionality, especially the dial wheel.

    Confirm the debugger is properly connected and passed through to WSL.

    Check the connection of wired peripherals.

Modifications

    Change sampleFreq in MahonyAHRS.c; this will affect the fusion result.

    Initialize a task for IMU in the FreeRTOS environment.