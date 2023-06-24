set(LIBRARY_NAME motor2040)

add_library(${LIBRARY_NAME} INTERFACE)

target_include_directories(${LIBRARY_NAME} INTERFACE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(${LIBRARY_NAME} INTERFACE
  pico_stdlib
  motor
  )