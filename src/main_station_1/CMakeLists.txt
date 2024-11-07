# Define the main_station_1 executable for the wheel encoder application
add_executable(main_station_1
    main_station_1.c
    ../wheel_encoder/wheel_encoder.c  # Add wheel_encoder.c to the motor project
    ../ultrasonic_sensor/ultrasonic_sensor.c # Add ultrasonic_sensor.c to the motor project
    ../motor/motor.c
    ../ir_sensor/line_following/line_following.c
)

# Link libraries for the wheel encoder executable
target_link_libraries(main_station_1
    pico_stdlib
    hardware_adc
    hardware_pwm
    hardware_gpio
    hardware_timer
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

# Add extra outputs for UF2, bin, and hex formats
pico_add_extra_outputs(main_station_1)

# Include the current directory for FreeRTOSConfig.h
target_include_directories(main_station_1 PRIVATE ${CMAKE_CURRENT_LIST_DIR})

# Enable USB output, disable UART output for the wheel encoder executable
pico_enable_stdio_usb(main_station_1 1)
pico_enable_stdio_uart(main_station_1 0)