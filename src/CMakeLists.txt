# Create the executable from your source files
add_executable(${PROJECT_NAME}
    main.c
    stepper.c
    l298n.c
    trapezoidal_profile.c
    sinusoidal_profile.c
    lowpass_filters.c
    controller_params.c

)

# Include header directory
target_include_directories(${PROJECT_NAME} PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}/../inc
)

# Link with Pico standard libraries
target_link_libraries(${PROJECT_NAME}
    pico_stdlib
    hardware_gpio
    hardware_pwm
    hardware_clocks
    pico_time
    pico_platform
    pico_stdlib
)

# Enable USB and UART output
pico_enable_stdio_usb(${PROJECT_NAME} 1)
pico_enable_stdio_uart(${PROJECT_NAME} 0)

# Generate UF2 file for flashing
pico_add_extra_outputs(${PROJECT_NAME})
