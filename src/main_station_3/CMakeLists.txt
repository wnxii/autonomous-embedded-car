add_executable(main_station_3_accel
        accel_with_client_socket.c
        )
target_compile_definitions(main_station_3_accel PRIVATE
        NO_SYS=0
        LWIP_SOCKET=1
        )
target_include_directories(main_station_3_accel PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        )
target_link_libraries(main_station_3_accel
        pico_cyw43_arch_lwip_sys_freertos
        pico_stdlib
        pico_lwip_iperf
        hardware_i2c
        FreeRTOS-Kernel-Heap4
        )
pico_add_extra_outputs(main_station_3_accel)
pico_enable_stdio_usb(main_station_3_accel 1)

add_executable(main_station_3_server
        server_socket.c
        )
target_compile_definitions(main_station_3_server PRIVATE
        NO_SYS=0
        LWIP_SOCKET=1
        )
target_include_directories(main_station_3_server PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        )
target_link_libraries(main_station_3_server
        pico_cyw43_arch_lwip_sys_freertos
        pico_stdlib
        pico_lwip_iperf
        FreeRTOS-Kernel-Heap4
        )
pico_add_extra_outputs(main_station_3_server)
pico_enable_stdio_usb(main_station_3_server 1)