add_executable(receiver
    receiver.c
)

target_compile_definitions(receiver PRIVATE
    NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
    LWIP_SOCKET=1       # we need the socket API (generally this would be in your lwipopts.h)
)

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(receiver
    pico_stdlib
    pico_cyw43_arch_lwip_sys_freertos
    pico_lwip_iperf
    FreeRTOS-Kernel
    FreeRTOS-Kernel-Heap4
    hardware_adc
    hardware_gpio
)

# Include directories for header files
target_include_directories(receiver PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
    ${PICO_SDK_PATH}/lib/lwip/src/include
    ${PICO_SDK_PATH}/lib/cyw43-driver/src
)

# Enable USB output, disable UART output
pico_enable_stdio_usb(receiver 1)
pico_enable_stdio_uart(receiver 0)

# Create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(receiver)
