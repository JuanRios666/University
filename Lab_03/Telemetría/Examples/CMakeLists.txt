add_executable(telemetria main.c)

# Pull in our pico_stdlib which pulls in commonly used features
target_link_libraries(telemetria PRIVATE pico_stdlib pico_mpu_9250 gps wifi)


pico_set_program_name(telemetria "main")
pico_set_program_version(telemetria "0.1")

pico_enable_stdio_usb(telemetria 1)
pico_enable_stdio_uart(telemetria 0)

# create map/bin/hex file etc.
pico_add_extra_outputs(telemetria)

# add url via pico_set_program_url
example_auto_set_url(telemetria)
