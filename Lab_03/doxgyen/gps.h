/**
  @file gps.h
  @brief Libreria del driver del GPS que permite la recuperación de las tramas de datos NMEA

    Este módulo usa interrupciones para recuperar las tramas de datos NMEA del GPS, y 
    las almacena en un buffer. Una vez el buffer esta lleno, se verifica si es la trama correcta
    y se extrae la latitud y longitud de la trama para ser enviadas al servidor

  @author Juan David Rios Rivera - Mario Alejandro Tabares Orjuela
  @date 07/05/2022

*/
#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"

#ifndef _gps_h
#define _gps_h
#define BUFFER_SIZE 256

extern bool new_data_available ;
extern uint8_t rx_buffer[];
extern size_t rx_buffer_length;
extern uint8_t sentence_buffer[];
extern size_t sentence_buffer_length;

/*************************************************************************************************/
/**
  @brief Inicia el módulo UART para la comunicación con el GPS
  En este caso usa el UART1, con un baudrate de 9600, 8 bits de datos, 1 bit de parada y sin paridad
  @returns Nothing

  Example:
  \verbatim
   \endverbatim

*/
void init_uart1();

/*************************************************************************************************/
/**
  @brief Función que se llama cuando se recibe una interrupción del UART

  @returns Boleana que indica si ya llego la trama completa

  Example:
  \verbatim
   \endverbatim

*/
bool uart_rx_handler();

/*************************************************************************************************/
/**
  @brief Extrae de la sentencia recuperada la latitud y la longitud

  @param nmea Sentencia NMEA a ser verificada
  @param lat Apuntador a la variable latitud donde se almacenará el valor de latitud
  @param lon Apuntador a la variable longitud donde se almacenará el valor de longitud

  @returns Nothing

*/
void extract_lat_long(char* nmea, float* lat, float* lon);

#endif