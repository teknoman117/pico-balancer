cmake_minimum_required(VERSION 3.13)

add_library(i2cdevlib STATIC I2Cdev.cpp MPU6050.cpp)

target_include_directories(i2cdevlib PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}>)

target_link_libraries(i2cdevlib PRIVATE pico_stdlib hardware_i2c)

target_link_libraries(i2cdevlib PUBLIC i2c_dma)