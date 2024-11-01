add_executable(line_following
        line_following.c
        )

target_include_directories(line_following PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        )

# pull in common dependencies
target_link_libraries(line_following pico_stdlib hardware_pwm hardware_adc FreeRTOS-Kernel-Heap4)

# enable usb output, disable uart output
pico_enable_stdio_usb(line_following 1)
pico_enable_stdio_uart(line_following 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(line_following)
