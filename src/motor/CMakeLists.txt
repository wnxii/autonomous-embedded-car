# Define the main executable for the motor control application
add_executable(motor
    motor.c
    ../main_wheel_encoder/wheel_encoder.c  # Add wheel_encoder.c to the motor project
    ../main_ultrasonic_sensor/ultrasonic_sensor.c # Add ultrasonic_sensor.c to the motor project
)

# Link libraries for the motor executable
target_link_libraries(motor
    pico_stdlib
    hardware_adc
    hardware_pwm
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

# Add extra outputs for UF2, bin, and hex formats
pico_add_extra_outputs(motor)

# Include the current directory for FreeRTOSConfig.h
target_include_directories(motor PRIVATE ${CMAKE_CURRENT_LIST_DIR})

# Enable USB output, disable UART output
pico_enable_stdio_usb(motor 1)
pico_enable_stdio_uart(motor 0)