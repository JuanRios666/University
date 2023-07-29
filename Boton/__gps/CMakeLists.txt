add_library(__gps __gps.c __gps.h)

target_link_libraries(__gps pico_stdlib hardware_uart)

target_include_directories(__gps PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")