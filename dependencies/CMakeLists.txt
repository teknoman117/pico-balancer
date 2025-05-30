cmake_minimum_required(VERSION 3.13)

# Add the correct FreeRTOS Kernel for our platform
if(PICO_PLATFORM STREQUAL "rp2040")
    add_subdirectory(FreeRTOS-Kernel/portable/ThirdParty/GCC/RP2040)
else()
    if (PICO_PLATFORM STREQUAL "rp2350-riscv")
        add_subdirectory(FreeRTOS-Kernel/portable/ThirdParty/Community-Supported-Ports/GCC/RP2350_RISC-V)
    else()
        add_subdirectory(FreeRTOS-Kernel/portable/ThirdParty/Community-Supported-Ports/GCC/RP2350_ARM_NTZ)
    endif()
endif()

# Add FreeRTOS kernel path for any dependencies that expect it
set(FREERTOS_KERNEL_PATH "${CMAKE_CURRENT_LIST_DIR}/FreeRTOS-Kernel")

# Add dma i2c library
add_subdirectory(pico-i2c-dma)

# Add i2cdevlib sources
add_subdirectory(i2cdevlib)
