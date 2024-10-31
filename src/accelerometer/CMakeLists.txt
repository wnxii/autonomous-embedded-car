add_executable(accelerometer
        test.c
        )

target_link_libraries(accelerometer
        pico_stdlib
        pico_cyw43_arch_none
        hardware_i2c
        )

pico_enable_stdio_usb(accelerometer 1)

pico_add_extra_outputs(accelerometer)
