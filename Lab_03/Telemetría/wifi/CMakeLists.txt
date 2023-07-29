add_library(wifi wifi.c wifi.h)

target_link_libraries(wifi pico_stdlib hardware_uart)

target_include_directories(wifi PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}")