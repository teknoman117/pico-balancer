cmake_minimum_required(VERSION 3.13)

add_executable(balancer
    balancer.cpp
    motors.cpp
)

pico_generate_pio_header(balancer
    ${CMAKE_CURRENT_LIST_DIR}/quadrature_encoder.pio
)

target_include_directories(balancer PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
)

target_compile_definitions(balancer PRIVATE
    WIFI_SSID=\"${WIFI_SSID}\"
    WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
    NO_SYS=0
    LWIP_SOCKET=1
)

target_link_libraries(balancer
    pico_cyw43_arch_lwip_sys_freertos
    pico_stdlib
    pico_lwip_mdns
    hardware_pwm
    hardware_pio
    i2cdevlib
    FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
)

pico_set_program_name(balancer "balancer")
pico_set_program_version(balancer "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(balancer 1)
pico_enable_stdio_usb(balancer 0)

pico_add_extra_outputs(balancer)