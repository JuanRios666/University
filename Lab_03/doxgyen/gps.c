/**
  @file gps.c
  @brief Módulo driver del GPS que permite la recuperación de las tramas de datos NMEA

    Este módulo usa interrupciones para recuperar las tramas de datos NMEA del GPS, y 
    las almacena en un buffer. Una vez el buffer esta lleno, se verifica si es la trama correcta
    y se extrae la latitud y longitud de la trama para ser enviadas al servidor

  @author Juan David Rios Rivera - Mario Alejandro Tabares Orjuela
  @date 07/05/2022

*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "gps.h"

#define RX_PIN_GPS 9                    /**< Define el pin RX del puerto uart1*/
#define TX_PIN_GPS 8                    /**< Define el pin TX del puerto uart1*/

#define UART_ID uart1                   /**< Define el puerto uart1*/
#define BAUD_RATE 9600                  /**< Define el baudrate del puerto uart1*/
#define DATA_BITS 8                     /**< Define el número de bits de datos del puerto uart1*/
#define STOP_BITS 1                     /**< Define el número de bits de parada del puerto uart1*/
#define PARITY    UART_PARITY_NONE      /**< Define el tipo de paridad del puerto uart1*/

bool new_data_available = false;        /**< Booleana que indica cuando se ha recibido una sentence NMEA completa*/
uint8_t rx_buffer[BUFFER_SIZE];         /*< Buffer que se llena con los datos que van llegando del GPS*/
size_t rx_buffer_length = 0;            /*< Recorre el buffer para llenarlo, con los datos del GPS*/
uint8_t sentence_buffer[BUFFER_SIZE];   /*< Buffer que almacena la sentence NMEA completa*/
size_t sentence_buffer_length = 0;      /*< Indicar cuando el buffer está lleno, agregando al final un \0 */

/*************************************************************************************************/
/**
  @brief Inicia el módulo UART para la comunicación con el GPS
  En este caso usa el UART1, con un baudrate de 9600, 8 bits de datos, 1 bit de parada y sin paridad
  @returns Nothing

  Example:
  \verbatim
   \endverbatim

*/
void init_uart1(){

    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(RX_PIN_GPS, GPIO_FUNC_UART);
    gpio_set_function(TX_PIN_GPS, GPIO_FUNC_UART);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
    
}

/*************************************************************************************************/
/**
  @brief Función que se llama cuando se recibe una interrupción del UART

  @returns Boleana que indica si ya llego la trama completa

  Example:
  \verbatim
   \endverbatim

*/
bool uart_rx_handler() {
    uint8_t data;
    while (uart_is_readable(UART_ID)) {
        data = uart_getc(UART_ID);
        rx_buffer[rx_buffer_length++] = data;
        if (data == '\n') {
            // End of sentence character received
            memcpy(sentence_buffer, rx_buffer, rx_buffer_length);
            sentence_buffer[rx_buffer_length - 1] = '\0';
            rx_buffer_length = 0;
            new_data_available = true;
        }
    }
}

/*************************************************************************************************/
/**
  @brief Extrae de la sentencia recuperada la latitud y la longitud

  @param nmea Sentencia NMEA a ser verificada
  @param lat Apuntador a la variable latitud donde se almacenará el valor de latitud
  @param lon Apuntador a la variable longitud donde se almacenará el valor de longitud

  @returns Nothing

*/

void extract_lat_long(char* nmea, float* lat, float* lon) {
    char* token;
    const char delimiter[2] = ",";
    int i = 0;
    // check if the sentence starts with "$GNGGA"
    if (strncmp(nmea, "$GNGGA", 6) == 0) {
        // use strtok to tokenize the sentence and extract the latitude and longitude
        printf("Received NMEA sentence: %s\n", nmea);
        token = strtok(nmea, delimiter);
        while (token != NULL && i < 10) {
            if (i == 2) {
                *lat = atof(token);
            }
            else if (i == 4) {
                *lon = atof(token);
            }
            i++;
            token = strtok(NULL, delimiter);
        }
        printf("Latitude: %f, Longitude: %f\n", *lat, *lon);
    }
}