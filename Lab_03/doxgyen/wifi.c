/**
  @file wifi.c
  @brief Módulo driver del WIFI que permite enviar tramas a un servidor

    Este módulo contiene las funciones para configurar el WIFI y enviar tramas a un servidor
    a través de una conexión TCP/IP.
    
  @author Juan David Rios Rivera - Mario Alejandro Tabares Orjuela
  @date 07/05/2022

*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "wifi.h"

#define UART_ID2 uart0                /**< UART a usar para la comunicación con el WIFI*/
#define BAUD_RATE 115200              /**< Define el Baudrate del UART*/
#define DATA_BITS 8                   /**< Define el número de bits de datos del UART*/
#define STOP_BITS 1                   /**< Define el número de bits de parada del UART*/
#define PARITY    UART_PARITY_NONE    /**< Define el tipo de paridad del UART*/

#define RX_PIN_WIFI 1                 /**< Define el pin RX del UART*/
#define TX_PIN_WIFI 0                 /**< Define el pin TX del UART*/

char SSID[] = "JDRios";               /**< SSID de la red WIFI*/
char password[] = "21656074";         /**< Contraseña de la red WIFI*/
char ServerIP[] = "192.168.43.142";   /**< IP del servidor*/
char Port[] = "80";                   /**< Puerto del servidor*/
char uart_command[50] = "";           /**< Buffer para almacenar los comandos AT*/
char buf[1024] = {0};                 /**< Buffer para almacenar los datos recibidos del servidor*/


/*************************************************************************************************/
/**
  @brief Inicia el módulo UART para la comunicación con el WIFI ESP8266
  En este caso es el UART0, con un baudrate de 115200, 8 bits de datos, 1 bit de parada y sin paridad
  
  @returns Nothing

*/
void init_uart0(){

    uart_init(UART_ID2, BAUD_RATE);
    gpio_set_function(RX_PIN_WIFI, GPIO_FUNC_UART);
    gpio_set_function(TX_PIN_WIFI, GPIO_FUNC_UART);

    uart_puts(UART_ID2, "+++");
    sleep_ms(1000);
    while (uart_is_readable(UART_ID2))
        uart_getc(UART_ID2);
    sleep_ms(2000);
    
}


/*************************************************************************************************/
/**
  @brief Inicia el módulo UART para la comunicación con el GPS

  @param data Trama de datos a enviar

  @returns Booleana que indica si se pudo enviar la trama

*/

bool send_sensor_values(const char *data)
{
    // Open connection
    sprintf(uart_command, "AT+CIPSTART=\"TCP\",\"%s\",%s", ServerIP, Port);
    sendCMD(uart_command, "OK");

    // Send data
    sendCMD("AT+CIPMODE=1", "OK");
    sleep_ms(100);
    sendCMD("AT+CIPSEND", ">");
    sleep_ms(200);


    uart_puts(UART_ID2, data);
    sleep_ms(100);
    uart_puts(UART_ID2, "+++"); // Look for ESP docs

    // Close connection
    sleep_ms(100);
    sendCMD("AT+CIPCLOSE", "OK");
    sleep_ms(100);
    sendCMD("AT+CIPMODE=0", "OK");

    return true;
}

/*************************************************************************************************/
/**
  @brief Envia un comando AT al modulo WIFI y espera una respuesta

  @param cmd Comando a enviar
  @param act Respuesta esperada

  @returns Boleana que indica si se envió el comando

*/

bool sendCMD(const char *cmd, const char *act)
{
    int i = 0;
    uint64_t t = 0;

    uart_puts(UART_ID2, cmd);
    uart_puts(UART_ID2, "\r\n");

    t = time_us_64();
    while (time_us_64() - t < 2500 * 1000)
    {
        while (uart_is_readable_within_us(UART_ID2, 2000))
        {
            buf[i++] = uart_getc(UART_ID2);
        }
        if (i > 0)
        {
            buf[i] = '\0';
            printf("%s\r\n", buf);
            if (strstr(buf, act) != NULL)
            {
                return true;
            }
            else
            {
                i = 0;
            }
        }
    }

    return false;
}

/*************************************************************************************************/
/**
  @brief Inicia la conexión con el WIFI, se establece el modo de operación y se conecta a la red

  @returns Nothing

*/

void connectToWifi()
{
    sendCMD("AT", "OK");
    sendCMD("AT+CWMODE=3", "OK");
    sprintf(uart_command, "AT+CWJAP=\"%s\",\"%s\"", SSID, password);
    sendCMD(uart_command, "OK");
}