/**
  @file __gps.h
  @brief Módulo driver del GPS que permite la recuperación de las tramas de datos NMEA

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
#define RX_PIN_GPS 9                    /**< Define el pin RX del puerto uart1*/
#define TX_PIN_GPS 8                    /**< Define el pin TX del puerto uart1*/

#define UART_ID uart1                   /**< Define el puerto uart1*/
#define BAUD_RATE 9600                  /**< Define el baudrate del puerto uart1*/
#define DATA_BITS 8                     /**< Define el número de bits de datos del puerto uart1*/
#define STOP_BITS 1                     /**< Define el número de bits de parada del puerto uart1*/
#define PARITY    UART_PARITY_NONE      /**< Define el tipo de paridad del puerto uart1*/


extern bool new_data_available ;
extern char rx_buffer[];
extern size_t rx_buffer_length;
extern char sentence_buffer[];
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

void decode(char gpsString[256], float * latitud, float * longitud);
size_t uart_read_line(uart_inst_t *uart, char *buffer, const size_t max_length);
bool is_correct(const char *message, const size_t length);
void send_with_checksum(uart_inst_t *uart, const char *message, const size_t length);

#endif