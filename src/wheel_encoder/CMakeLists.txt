# Define the main executable for the wheel encoder application
add_executable(wheel_encoder
    wheel_encoder.c
)

# Link libraries for the wheel encoder executable
target_link_libraries(wheel_encoder
    pico_stdlib
    hardware_gpio
    hardware_timer
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
)

# Add extra outputs for UF2, bin, and hex formats
pico_add_extra_outputs(wheel_encoder)

# Include the current directory for FreeRTOSConfig.h
target_include_directories(wheel_encoder PRIVATE ${CMAKE_CURRENT_LIST_DIR})

# Enable USB output, disable UART output for the wheel encoder executable
pico_enable_stdio_usb(wheel_encoder 1)
pico_enable_stdio_uart(wheel_encoder 0)