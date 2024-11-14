# Define the main_station_2 executable for the wheel encoder application
add_executable(main_station_2
    main_station_2.c
    ../wheel_encoder/wheel_encoder.c  # Add wheel_encoder.c to the motor project
    #../ultrasonic_sensor/ultrasonic_sensor.c # Add ultrasonic_sensor.c to the motor project
    ../motor/motor.c
    ../wifi/barcode_client_socket/barcode_client_socket.c
    ../ir_sensor/barcode_scanner/barcode_scanner.c
    ../ir_sensor/line_following/line_following.c
)

target_compile_definitions(main_station_2 PRIVATE
        NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
        LWIP_SOCKET=1       # we need the socket API (generally this would be in your lwipopts.h)
        )

# Link libraries for the wheel encoder executable
target_link_libraries(main_station_2
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
pico_add_extra_outputs(main_station_2)

# Include the current directory for FreeRTOSConfig.h
target_include_directories(main_station_2 PRIVATE ${CMAKE_CURRENT_LIST_DIR})

# Enable USB output, disable UART output for the wheel encoder executable
pico_enable_stdio_usb(main_station_2 1)
pico_enable_stdio_uart(main_station_2 0)