cmake_minimum_required(VERSION 3.12)

# rest of your project
add_executable(pico_lorawan_send_data_gps
    main.c
)

target_link_libraries(pico_lorawan_send_data_gps pico_lorawan hardware_adc __gps hardware_rtc)

# enable usb output, disable uart output
pico_enable_stdio_usb(pico_lorawan_send_data_gps 1)
pico_enable_stdio_uart(pico_lorawan_send_data_gps 0)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(pico_lorawan_send_data_gps)
