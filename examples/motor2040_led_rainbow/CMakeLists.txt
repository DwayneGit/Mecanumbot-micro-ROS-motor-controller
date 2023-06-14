cmake_minimum_required(VERSION 3.12)
set(OUTPUT_NAME motor2040_led_rainbow)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(${OUTPUT_NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(${OUTPUT_NAME} 
        motor2040_led_rainbow.c
        )

target_link_libraries(${OUTPUT_NAME}
        pico_stdlib
        motor2040
        button
        )

# enable usb output
pico_enable_stdio_usb(${OUTPUT_NAME} 1)

pico_add_extra_outputs(${OUTPUT_NAME})
