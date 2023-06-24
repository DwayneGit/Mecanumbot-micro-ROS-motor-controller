cmake_minimum_required(VERSION 3.12)

set(PROJECT micro_ros_motor_ctrl)

# set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT})
# set(SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)

# set(SOURCES
#     micro_ros_motor_ctrl.c
#     micro_ros_motor_ctrl_multi_threaded.c
#     ${CMAKE_CURRENT_SOURCE_DIR}/examples/multithread_publisher_subscriber/main.c
#     ${CMAKE_CURRENT_SOURCE_DIR}/examples/motor2040_led_rainbow/main.c
#     ${CMAKE_CURRENT_SOURCE_DIR}/examples/motor2040_single_motor/main.c
#     ${SRC_DIR}/pwm.c
#     ${SRC_DIR}/events.c
#     ${SRC_DIR}/bimotor.c
#     ${SRC_DIR}/mecanumbot.c
#     ${SRC_DIR}/pico_uart_transports.c
# )

# set (INCLUDES   
#     ${INCLUDE_DIR}/pwm.h
#     ${INCLUDE_DIR}/events.h
#     ${INCLUDE_DIR}/bimotor.h
#     ${INCLUDE_DIR}/mecanumbot.h
#     ${INCLUDE_DIR}/pico_uart_transports.h
# )

include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(${PROJECT} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

# link_directories($ENV{MICRO_ROS_PICO_SDK_PATH}/libmicroros)
# add_executable(${PROJECT}
    # ${SOURCES}
    # ${INCLUDES}
# )

# target_link_libraries(${PROJECT}
    # microros
    # motor2040
    # pico_stdlib
    # pico_multicore
    # hardware_pwm
    # hardware_dma
# )

# target_include_directories(${PROJECT} PUBLIC
#     $ENV{MICRO_ROS_PICO_SDK_PATH}/libmicroros/include 
# )

# target_compile_definitions(${PROJECT} PUBLIC
#     PICO_MAX_SHARED_IRQ_HANDLERS=6
#     PICO_STDIO_USB_LOW_PRIORITY_IRQ=26
# )

SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
# SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")


# Configure Pico
# pico_enable_stdio_usb(${PROJECT} 1)
# pico_enable_stdio_uart(${PROJECT} 0)
# add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
# add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
# add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2
# pico_add_extra_outputs(${PROJECT})

add_subdirectory(libraries)
add_subdirectory(examples)