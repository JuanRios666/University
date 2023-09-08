/**
  @file __gps.h
  @brief Módulo driver del GPS que permite la recuperación de las tramas de datos NMEA

    Este módulo usa interrupciones para recuperar las tramas de datos NMEA del GPS, y 
    las almacena en un buffer. Una vez el buffer esta lleno, se verifica si es la trama correcta
    y se extrae la latitud y longitud de la trama para ser enviadas al servidor

  @author Juan David Rios Rivera
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

void init_uart1();
bool uart_rx_handler();
void extract_lat_long(char* nmea, float* lat, float* lon);
bool decode(char gpsString[256], float * latitud, float * longitud, uint16_t *time);
size_t uart_read_line(uart_inst_t *uart, char *buffer, const size_t max_length);
bool is_correct(const char *message, const size_t length);
void send_with_checksum(uart_inst_t *uart, const char *message, const size_t length);
bool is_Colombia(float * lat, float * lon);
float toRadians(float degrees);
float calcularDistancia(float *lat1, float *lon1, float *lat2, float *lon2);

#endif