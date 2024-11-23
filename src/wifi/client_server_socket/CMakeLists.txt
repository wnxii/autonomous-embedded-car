add_executable(client_server_socket
        client_server_socket.c
        )
target_compile_definitions(client_server_socket PRIVATE
        NO_SYS=0            # don't want NO_SYS (generally this would be in your lwipopts.h)
        LWIP_SOCKET=1       # we need the socket API (generally this would be in your lwipopts.h)
        )
target_include_directories(client_server_socket PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/.. # for our common lwipopts
        )
target_link_libraries(client_server_socket
        pico_cyw43_arch_lwip_sys_freertos
        pico_stdlib
        pico_lwip_iperf
        hardware_pwm 
        hardware_adc
        FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
        )
pico_add_extra_outputs(client_server_socket)
pico_enable_stdio_usb(client_server_socket 1)