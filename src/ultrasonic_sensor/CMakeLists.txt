# Define the main executable for the wheel encoder application
add_executable(ultrasonic_sensor
    ultrasonic_sensor.c
)

# Link libraries for the wheel encoder executable
target_link_libraries(ultrasonic_sensor
    pico_stdlib
    hardware_gpio
    hardware_timer
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

# Add extra outputs for UF2, bin, and hex formats
pico_add_extra_outputs(ultrasonic_sensor)

# Include the current directory for FreeRTOSConfig.h
target_include_directories(ultrasonic_sensor PRIVATE ${CMAKE_CURRENT_LIST_DIR})

# Enable USB output, disable UART output for the wheel encoder executable
pico_enable_stdio_usb(ultrasonic_sensor 1)
pico_enable_stdio_uart(ultrasonic_sensor 0)