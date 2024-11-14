add_executable(accelerometer
        accelerometer.c
        )

target_include_directories(accelerometer PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(accelerometer
        pico_stdlib
        pico_cyw43_arch_lwip_threadsafe_background
        pico_lwip_iperf
        hardware_i2c
        )

pico_enable_stdio_usb(accelerometer 1)

pico_add_extra_outputs(accelerometer)