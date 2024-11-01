add_executable(barcode_scanner
        barcode_scanner.c
        )

target_include_directories(barcode_scanner PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        )

# pull in common dependencies
target_link_libraries(barcode_scanner pico_stdlib hardware_pwm hardware_adc FreeRTOS-Kernel-Heap4)

# enable usb output, disable uart output
pico_enable_stdio_usb(barcode_scanner 1)
pico_enable_stdio_uart(barcode_scanner 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(barcode_scanner)
