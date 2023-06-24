set(LIBRARY_NAME motor)

add_library(${LIBRARY_NAME} INTERFACE)

target_sources(${LIBRARY_NAME} INTERFACE
  ${CMAKE_CURRENT_LIST_DIR}/common.c
  ${CMAKE_CURRENT_LIST_DIR}/pwm.c
  ${CMAKE_CURRENT_LIST_DIR}/motor.c
  ${CMAKE_CURRENT_LIST_DIR}/motor_state.c
  )

target_include_directories(${LIBRARY_NAME} INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(${LIBRARY_NAME} INTERFACE
  pico_stdlib
  hardware_pwm
  )