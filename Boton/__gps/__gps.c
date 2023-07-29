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
#include "__gps.h"

bool new_data_available = false;        /**< Booleana que indica cuando se ha recibido una sentence NMEA completa*/
char rx_buffer[BUFFER_SIZE];         /*< Buffer que se llena con los datos que van llegando del GPS*/
size_t rx_buffer_length = 0;            /*< Recorre el buffer para llenarlo, con los datos del GPS*/
char sentence_buffer[BUFFER_SIZE];   /*< Buffer que almacena la sentence NMEA completa*/
size_t sentence_buffer_length = 0;      /*< Indicar cuando el buffer está lleno, agregando al final un \0 */

const uint8_t DATABITS = 8;
const uint8_t STARTBITS = 1;
const uint8_t STOPBITS = 1;
const uint32_t GPS_BAUDRATE = 9600;

// Calculate the delay between bytes
const uint8_t BITS_PER_BYTE = STARTBITS + DATABITS + STOPBITS;
const uint32_t MICROSECONDS_PER_SECOND = 1000000;
const uint32_t GPS_DELAY = BITS_PER_BYTE * MICROSECONDS_PER_SECOND / GPS_BAUDRATE;

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
    uart_set_translate_crlf(UART_ID, false);
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
            printf("TEXT: %s\n",sentence_buffer);
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
    if (strncmp(nmea, "$GNRMC", 6) == 0) {
        // use strtok to tokenize the sentence and extract the latitude and longitude
        printf("Received NMEA sentence: %s\n", nmea);
        token = strtok(nmea, delimiter);
        while (token != NULL && i < 10) {
            if (i == 3) {
                *lat = atof(token);
            }
            else if (i==4){
                if(token[0]=='S'){
                    *lat *= -1;
                }
            }
            else if (i == 5) {
                *lon = atof(token);
            }
            else if (i==6){
                if(token[0]=='W'){
                    *lon *=-1;
                }
            }
            i++;
            token = strtok(NULL, delimiter);
        }
        printf("Latitude: %f, Longitude: %f\n", *lat, *lon);
    }
}

/**
 * @brief Decodifica la trama del GPS.
 *
 * Esta función decodifica la trama del GPS y obtiene la latitud y longitud. Y no retorna nada. por lo que se debe usar punteros para obtener los valores.
 *
 * @param char gpsString[256] este es el arreglo que contiene la trama del GPS.
 * @param float * latitud este es el puntero a la variable que contiene la latitud.
 * @param float * longitud este es el puntero a la variable que contiene la longitud.
 */
// Define uart properties for the ESP
void decode(char gpsString[256], float * latitud, float * longitud) { // Function to decode the GPS string
  char *token = strtok(gpsString, ","); // Split the string by commas
  char *token_old, *latitude = 0, *longitude = 0; // Variables to store the latitude and longitude
/**
 * @brief itera sobre la trama del GPS.
 *
 * Este while itera sobre la trama del GPS y obtiene la latitud y longitud.
 *
 */
  while (token != NULL) { // Iterate over the string
    if (strcmp(token, "N") == 0) { // If the token is N, the next token is the latitude
      latitude = token_old; // Store the latitude
    }
    else if (strcmp(token, "W") == 0) { // If the token is W, the next token is the longitude
      longitude = token_old; // Store the longitude
    }
    token_old = token; // Store the current token
    token = strtok(NULL, ","); // Get the next token
  }

  if (latitude != NULL && longitude != NULL){ // If the latitude and longitude are not null, decode them
    float lat = atof(latitude); // Convert the latitude and longitude to float
    int aux = (int)lat/100; // Get the integer part of the latitude and longitude
    *latitud = (lat-(100*aux))/60 + (float)aux; // Convert the latitude and longitude to decimal degrees
    
    float lon = -atof(longitude); // Convert the latitude and longitude to float
    int aux2 = (int)lon/100; // Get the integer part of the latitude and longitude
    *longitud = (lon-(100*aux2))/60 + (float)aux2; // Convert the latitude and longitude to decimal degrees
  }
  
}

/**
 * @brief Lee una linea del uart.
 *
 * Esta función lee una linea del uart y la guarda en un arreglo. 
 *
 * @param uart_inst_t *uart este es el uart que se va a leer.
 * @param char *buffer este es el arreglo donde se va a guardar la linea.
 * @param const size_t max_length este es el tamaño maximo del arreglo.
 * @return i retorna el tamaño de la linea.
 */
// Function to read a line from the uart
size_t uart_read_line(uart_inst_t *uart, char *buffer, const size_t max_length){ // Function to read a line from the uart
    size_t i;
    // Receive the bytes with as much delay as possible without missing data
    buffer[0] = uart_getc(uart);
    for(i = 1;i < max_length - 1 && buffer[i - 1] != '\n';i++){
        sleep_us(GPS_DELAY);
        buffer[i] = uart_getc(uart);
    }

    // End the string with a terminating 0 and return the length
    buffer[i] = '\0';
    new_data_available = true;
    return i;
}

/**
 * @brief Si la trama es correcta.
 *
 * Esta función verifica si la trama del GPS es correcta.
 *
 * @param const char *message este es el arreglo que contiene la trama.
 * @param const size_t length este es el tamaño de la trama.
 * @return strncmp(checksum, message + i + 1, 2) == 0 retorna true si la trama es correcta.
 */

bool is_correct(const char *message, const size_t length){  // Function to check if the message is correct
    char sum = 0;
    char checksum[3];
    size_t i;

    // The message should start with $ and end with \r\n
    if(message[0] != '$' || message[length - 1] != '\n' || message[length - 2] != '\r'){
        return false;
    }  
    for(i = 1;i < length && message[i] != '*';i++){
        sum ^= message[i];
    }
    if(message[i] != '*'){
        return false;
    }
    for(size_t i = 0;i < 2;i++){
        if(sum % 16 < 10){
            checksum[1 - i] = '0' + sum % 16;
        }else{
            checksum[1 - i] = 'A' + sum % 16 - 10;
        }
        sum >>= 4;
    }
    checksum[2] = '\0';

    return strncmp(checksum, message + i + 1, 2) == 0;
}

/**
 * @brief Envia un mensaje con checksum.
 *
 * Esta función envia un mensaje con checksum para verificar que la trama sea correcta. Y no retorna nada. por lo que se debe usar punteros para obtener los valores.
 *
 * @param uart_inst_t *uart este es el uart que se va a usar.
 * @param const char *message este es el arreglo que contiene la trama.
 * @param const size_t length este es el tamaño de la trama.
 */
// Function to send a message with checksum
void send_with_checksum(uart_inst_t *uart, const char *message, const size_t length){ 
    char sum = 0;
    char checksum[3];

    // Calcute the checksum
    for(size_t i = 0;i < length;i++){
        sum ^= message[i];
    }

    // Convert the checksum to a hexadecimal string
    for(size_t i = 0;i < 2;i++){
        if(sum % 16 < 10){
            checksum[1 - i] = '0' + sum % 16;
        }else{
            checksum[1 - i] = 'A' + sum % 16 - 10;
        }
        sum >>= 4;
    }
    checksum[2] = '\0';

    // Send the message to the GPS in the expected format
    uart_putc_raw(uart, '$');
    uart_puts(uart, message);
    uart_putc(uart, '*');
    uart_puts(uart, checksum);
    uart_puts(uart, "\r\n");
}