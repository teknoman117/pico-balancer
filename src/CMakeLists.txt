cmake_minimum_required(VERSION 3.13)

add_executable(pico-balancer
    pico-balancer.cpp
)

target_include_directories(pico-balancer PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>
)

target_compile_definitions(pico-balancer PRIVATE
    WIFI_SSID=\"${WIFI_SSID}\"
    WIFI_PASSWORD=\"${WIFI_PASSWORD}\"
    NO_SYS=0
    LWIP_SOCKET=1
)

target_link_libraries(pico-balancer
    pico_cyw43_arch_lwip_sys_freertos
    pico_stdlib
    pico_lwip_mdns
    hardware_i2c
    i2cdevlib
    FreeRTOS-Kernel-Heap4 # FreeRTOS kernel and dynamic heap
)

pico_set_program_name(pico-balancer "pico-balancer")
pico_set_program_version(pico-balancer "0.1")

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(pico-balancer 1)
pico_enable_stdio_usb(pico-balancer 0)

pico_add_extra_outputs(pico-balancer)