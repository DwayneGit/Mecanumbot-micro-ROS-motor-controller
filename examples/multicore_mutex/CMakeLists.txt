cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(mutex_example C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(mutex_example
    mutex_example.c
)
target_link_libraries(mutex_example
    pico_stdlib
    pico_multicore
)

# Configure Pico
pico_enable_stdio_usb(mutex_example 1)
pico_enable_stdio_uart(mutex_example 1)

# Generate UF2
pico_add_extra_outputs(mutex_example)
