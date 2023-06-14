set(LIBRARY_NAME motor2040)

add_library(${LIBRARY_NAME} 
  common.c
  common.h
  motor.c
  motor.h
  motor_state.c
  motor_state.h
  pwm.h
  pwm.c
  motor2040.h
  )

target_include_directories(${LIBRARY_NAME} PUBLIC ${CMAKE_CURRENT_LIST_DIR})

pico_generate_pio_header(${LIBRARY_NAME} ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio)

target_link_libraries(${LIBRARY_NAME} PUBLIC
  pico_stdlib
  # pwm_cluster
  hardware_pio
  hardware_pwm
  )