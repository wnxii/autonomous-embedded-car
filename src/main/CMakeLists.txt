# Define the main executable for the wheel encoder application
add_executable(main
    main.c
    ../wheel_encoder/wheel_encoder.c  # Add wheel_encoder.c to the motor project
    ../motor/motor.c
    ../wifi/barcode_client_socket/barcode_client_socket.c
    ../ir_sensor/barcode_scanner/barcode_scanner.c
    ../ir_sensor/line_following/line_following.c
    ../ultrasonic_sensor/ultrasonic_sensor.c
)

target_compile_definitions(main PRIVATE
        NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
        LWIP_SOCKET=1       # we need the socket API (generally this would be in your lwipopts.h)
        )

# Link libraries for the wheel encoder executable
target_link_libraries(main
    pico_stdlib
    hardware_adc
    hardware_pwm
    hardware_gpio
    hardware_timer
    pico_cyw43_arch_lwip_sys_freertos
    pico_lwip_iperf
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

# Add extra outputs for UF2, bin, and hex formats
pico_add_extra_outputs(main)

# Include the current directory for FreeRTOSConfig.h
target_include_directories(main PRIVATE ${CMAKE_CURRENT_LIST_DIR})

# Enable USB output, disable UART output for the wheel encoder executable
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)
