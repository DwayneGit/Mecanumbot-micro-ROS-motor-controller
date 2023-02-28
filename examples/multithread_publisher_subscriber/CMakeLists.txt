cmake_minimum_required(VERSION 3.12)

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(multithread_publisher_subscriber C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(multithread_publisher_subscriber
    main.c
)
target_link_libraries(multithread_publisher_subscriber
    pico_stdlib
    pico_multicore
)

# Configure Pico
pico_enable_stdio_usb(multithread_publisher_subscriber 1)
pico_enable_stdio_uart(multithread_publisher_subscriber 1)

# Generate UF2
pico_add_extra_outputs(multithread_publisher_subscriber)
