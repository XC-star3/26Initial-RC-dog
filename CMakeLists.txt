cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME dog_leg_test_SDK)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME} C CXX ASM)
message("Build type: " ${CMAKE_BUILD_TYPE})

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

set(USER_INCLUDE_DIRS
    ${CMAKE_SOURCE_DIR}/User_File/algorithm
    ${CMAKE_SOURCE_DIR}/User_File/BSP/CAN
    ${CMAKE_SOURCE_DIR}/User_File/BSP/IMU
    ${CMAKE_SOURCE_DIR}/User_File/BSP/IMU/inc
    ${CMAKE_SOURCE_DIR}/User_File/BSP/RGB
    ${CMAKE_SOURCE_DIR}/User_File/BSP/USB
    ${CMAKE_SOURCE_DIR}/User_File/motor/MWSDK
    ${CMAKE_SOURCE_DIR}/User_File/motor/motor_DM
    ${CMAKE_SOURCE_DIR}/User_File/motor/motor_LZ
    ${CMAKE_SOURCE_DIR}/User_File/remote
    ${CMAKE_SOURCE_DIR}/User_File/task
    ${CMAKE_SOURCE_DIR}/User_File/task/arm
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    ${CMAKE_SOURCE_DIR}/User_File/BSP/CAN/bsp_fdcan.cpp
    ${CMAKE_SOURCE_DIR}/User_File/BSP/IMU/BMI088driver.c
    ${CMAKE_SOURCE_DIR}/User_File/BSP/IMU/BMI088Middleware.c
    ${CMAKE_SOURCE_DIR}/User_File/BSP/IMU/imu_task.cpp
    ${CMAKE_SOURCE_DIR}/User_File/BSP/RGB/board_rgb.cpp
    ${CMAKE_SOURCE_DIR}/User_File/BSP/USB/debug_uart.cpp
    ${CMAKE_SOURCE_DIR}/User_File/BSP/USB/vofa_pid.cpp
    ${CMAKE_SOURCE_DIR}/User_File/BSP/system_startup_hooks.c
    ${CMAKE_SOURCE_DIR}/User_File/motor/MWSDK/MWMotor.c
    ${CMAKE_SOURCE_DIR}/User_File/motor/motor_DM/motor_DM.cpp
    ${CMAKE_SOURCE_DIR}/User_File/motor/motor_LZ/motor_LZ.cpp
    ${CMAKE_SOURCE_DIR}/User_File/remote/sbus.c
    ${CMAKE_SOURCE_DIR}/User_File/task/arm/arm_motor_task.cpp
    ${CMAKE_SOURCE_DIR}/User_File/task/control_task.cpp
    ${CMAKE_SOURCE_DIR}/User_File/task/motor_task.cpp
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    ${USER_INCLUDE_DIRS}
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    ARM_MATH_CM7
    __FPU_PRESENT=1U
    ARM_MATH_CHECK
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

if(CMAKE_CXX_COMPILER_LOADED)
    target_link_libraries(${CMAKE_PROJECT_NAME} stdc++ supc++)
endif()
