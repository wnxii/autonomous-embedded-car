add_executable(dashboard
    dashboard.c
)

target_include_directories(dashboard PRIVATE
    ${CMAKE_CURRENT_LIST_DIR}
)

target_link_libraries(dashboard
    pico_cyw43_arch_lwip_threadsafe_background
    pico_lwip_http
    pico_stdlib
    hardware_adc
)

pico_enable_stdio_usb(dashboard TRUE)
pico_enable_stdio_uart(dashboard FALSE)

pico_add_extra_outputs(dashboard)