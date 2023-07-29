#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"

#ifndef wifi_h
#define wifi_h

void init_uart0();
bool send_sensor_values(const char *data);
bool sendCMD(const char *cmd, const char *act);
void connectToWifi();

#endif