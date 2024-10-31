add_executable(accelerometer_local
        test.c
        )

target_link_libraries(accelerometer_local
        pico_stdlib
        pico_cyw43_arch_none
        hardware_i2c
        )

pico_enable_stdio_usb(accelerometer 1)

pico_add_extra_outputs(accelerometer)

add_executable(accelerometer_socket
        test_socket.c
        )
target_compile_definitions(accelerometer_socket PRIVATE
        NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
        LWIP_SOCKET=1       # we need the socket API (generally this would be in your lwipopts.h)
        )
target_include_directories(accelerometer_socket PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
        )
target_link_libraries(accelerometer_socket
        pico_cyw43_arch_lwip_sys_freertos
        pico_stdlib
        pico_lwip_iperf
        hardware_i2c
        FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
        )

pico_add_extra_outputs(accelerometer_socket)
pico_enable_stdio_usb(accelerometer_socket 1)