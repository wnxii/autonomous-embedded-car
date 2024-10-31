add_executable(motor
    motor.c
)

target_link_libraries(motor
    pico_stdlib
    hardware_adc
    hardware_pwm
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

pico_add_extra_outputs(motor)

# Include the current directory for FreeRTOSConfig.h
target_include_directories(motor PRIVATE ${CMAKE_CURRENT_LIST_DIR})

# Enable USB output, disable UART output
pico_enable_stdio_usb(motor 1)
pico_enable_stdio_uart(motor 0)
