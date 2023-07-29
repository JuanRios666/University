/**
  @file wifi.h
  @brief Libreria del driver del WIFI que permite enviar tramas a un servidor

    Este módulo contiene las funciones para configurar el WIFI y enviar tramas a un servidor
    a través de una conexión TCP/IP.
    
  @author Juan David Rios Rivera - Mario Alejandro Tabares Orjuela
  @date 07/05/2022

*/

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