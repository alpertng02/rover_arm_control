cmake_minimum_required(VERSION 3.13)  # Specify the minimum CMake version required

find_program(PICOTOOL picotool)

# Include the Pico SDK
include(pico_sdk_import.cmake)

# Define the project name, version, and supported languages
project(rover_arm_control VERSION 1.0 LANGUAGES C CXX ASM)

# Set the C and C++ standards
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

add_compile_options(-Wall -Wextra -Werror -Wno-unused-parameter -Wno-unused-variable) 

# Initialize the Pico SDK
pico_sdk_init()

# Add subdirectories
add_subdirectory(src)

add_custom_target (flash ${PICOTOOL} load -f ${PROJECT_NAME}.uf2 && ${PICOTOOL} reboot DEPENDS ${PROJECT_NAME})

