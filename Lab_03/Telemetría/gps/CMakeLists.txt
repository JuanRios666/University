add_library(gps gps.c gps.h)

target_link_libraries(gps pico_stdlib hardware_uart)

target_include_directories(gps PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")