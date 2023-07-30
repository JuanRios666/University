/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 * This example uses OTAA to join the LoRaWAN network and then sends the 
 * internal temperature sensors value up as an uplink message periodically 
 * and the first byte of any uplink messages received controls the boards
 * built-in LED.
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "hardware/adc.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "hardware/rtc.h"
#include "pico/util/datetime.h"

#include "pico/stdlib.h"
#include "pico/lorawan.h"
#include "tusb.h"
#include "__gps.h"
#include "config.h"

#define WAITFORWAKEUP()  __asm volatile ("wfi") //Modo bajo consumo
#define INT_BUTTON 6
#define INT_GPS 13
#define LED_PIN 25
#define TIMEOUT 5000

//Variables de interrupciones
volatile bool button_pressed = false;
volatile bool int_pressed = false;
static volatile bool fired = false;

// Configuraciones para el modulo LoRA
const struct lorawan_sx12xx_settings sx12xx_settings = {
    .spi = {
        .inst = spi1,
        .mosi = 11,
        .miso = 12,
        .sck  = 10,
        .nss  = 3
    },
    .reset = 15,
    .busy = 2,
    // sx127x would use dio0 pin, and sx126x dont use it 
    // .dio0  = 7,
    .dio1  = 20
};

// Configuraciones del modo ABP
struct lorawan_abp_settings abp_settings = {
    .device_address = LORAWAN_DEV_ADDR,
    .network_session_key = LORAWAN_NETWORK_SESSION_KEY,
    .app_session_key = LORAWAN_APP_SESSION_KEY,
    .channel_mask = LORAWAN_CHANNEL_MASK
};


void wait_for_response(){
    char message[256];
    const size_t length = uart_read_line(uart1, message, sizeof(message));
    printf("%s\n", message);

}

static void alarm_callback(void) {
    datetime_t t = {0};
    rtc_get_datetime(&t);
    char datetime_buf[256];
    char *datetime_str = &datetime_buf[0];
    datetime_to_str(datetime_str, sizeof(datetime_buf), &t);
    printf("Alarm Fired At %s\n", datetime_str);
    stdio_flush();
    fired = true;
}

void calculate_Checksum(uint8_t* data, uint8_t endPos){
    uint8_t CK_A = 0;
    uint8_t CK_B = 0;
    for(uint8_t i = 2;i< endPos;i++){   
        CK_A = CK_A + data[i];
        CK_B = CK_B + CK_A;
    }
    data[endPos] = CK_A;
    data[endPos+1] = CK_B;
}


// Variables para recibir datos por LoRa
int receive_length = 0;
uint8_t receive_buffer[242];
uint8_t receive_port = 0;
char dev_eui[17];

// Funciones usadas en el main
void gpio_int_handler(uint gpio, uint32_t events);
void send_gps(float *latitude, float *longitude, uint32_t *last_message_time, uint8_t *count);


int main( void )
{
    const char CONFIGURATIONS[] = "PMTK103"; 
    const char CONFIGURATIONS2[] = "PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0";
    /*uint8_t config[10] = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x00, 0x01, 0x00, 0x00}; //RXM
    uint8_t config2[16] = {0xB5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00};//PMG
    calculate_Checksum(config, sizeof(config)-2);
    calculate_Checksum(config2, sizeof(config2)-2);*/

    // ---------------------- initialize the board ---------------------------
    lorawan_default_dev_eui(dev_eui);
    abp_settings.device_address = dev_eui;
    stdio_init_all();
    sleep_ms(4000);
    printf("DEV_EUI: %s\n", abp_settings.device_address);
    printf("APP_EUI: %s\n", abp_settings.app_session_key);
    printf("APP_KEY: %s\n", abp_settings.network_session_key);

    printf("Pico LoRaWAN - Button Panic and GPS\n\n");
    
    init_uart1();
    //uart_write_blocking(UART_ID, config2, sizeof(config2));
    //wait_for_response();
    send_with_checksum(UART_ID, CONFIGURATIONS, sizeof(CONFIGURATIONS));
    wait_for_response();

    // ---------------------- initialize the LoRaWAN stack---------------------------
    printf("Initilizating LoRaWAN ... ");
    if (lorawan_init_abp(&sx12xx_settings, LORAWAN_REGION, &abp_settings) < 0) {
        printf("failed!!!\n");
        while (1) {
            tight_loop_contents();
        }
    } else {
        printf("success!\n");
    }

    //----------Interrupcion del Botón de Pánico-----------
    gpio_init(INT_BUTTON);                      //inicio el boton de la interrupción
    gpio_set_dir(INT_BUTTON, GPIO_IN);          //seteo el boton como entrada
    gpio_pull_up(INT_BUTTON);                   //activo la resistencia de pull up
    gpio_set_irq_enabled_with_callback(INT_BUTTON, GPIO_IRQ_EDGE_RISE, true, &gpio_int_handler);
    irq_set_enabled(INT_BUTTON, true);
    irq_set_priority(INT_BUTTON, 0);

    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);


    uint8_t counter = 0;
    uint32_t last_message_time = 0;

    datetime_t t = {
        .year  = 2023,
        .month = 07,
        .day   = 27,
        .dotw  = 4, // 0 is Sunday, so 3 is Wednesday
        .hour  = 12,
        .min   = 00,
        .sec   = 00
    };

    rtc_init();
    rtc_set_datetime(&t);

    // Alarm once a minute
    datetime_t alarm = {
        .year  = -1,
        .month = -1,
        .day   = -1,
        .dotw  = -1,
        .hour  = 12,
        .min   = 00,
        .sec   = 00
    };

    rtc_set_alarm(&alarm, &alarm_callback);
    
    while (1) {
        lorawan_process();
        if(button_pressed || fired){ 
            gpio_put(PICO_DEFAULT_LED_PIN, 1);    
            const size_t length = uart_read_line(UART_ID, rx_buffer, BUFFER_SIZE);
            //send_gps(&latitude, &longitude, &last_message_time, &counter);
            if(!is_correct(rx_buffer, length)){
                new_data_available = false;
            }
            
            if(new_data_available){        //ha llegado toda una sentencia GPS
                if (strncmp(rx_buffer, "$GNRMC", strlen("$GNRMC")) == 0 || strncmp(rx_buffer, "$GNGGA", strlen("$GNGGA")) == 0){
                    //printf("%s\n", rx_buffer);
                    float latitud = 0, longitud = 0;
                    decode(rx_buffer, &latitud, &longitud);
                    if((latitud > 0.0) && (longitud < 0.0)){ //Espera hasta encontrar una coordenada válida
                        send_gps(&latitud, &longitud, &last_message_time, &counter);
                    }
                }
                new_data_available = false;
            }
        }else{
            WAITFORWAKEUP();
        }        
        
    }
    return 0;
}

void gpio_int_handler(uint gpio, uint32_t events) {
    if (gpio == INT_BUTTON && (events & GPIO_IRQ_EDGE_RISE)) {
        button_pressed = true;
    }
}

void send_gps(float *latitude, float *longitude, uint32_t *last_message_time, uint8_t *count){
        
        uint32_t now = to_ms_since_boot(get_absolute_time());
        if ((now - *last_message_time) > TIMEOUT) {
            uint8_t len = snprintf(NULL,0, "1, %f, %f", *latitude, *longitude);
            char message[len];
            sprintf(message, "%d, %f, %f", button_pressed, *latitude, *longitude);
            
             printf("sending unconfirmed message %s ...  No: %d ", message, *count);
             
            if (lorawan_send_unconfirmed(message, strlen(message), 2) < 0) {
                printf("failed!!!\n");
            } else{
                printf("SUCCESS!\n");
                printf("---------PANIC SEND---------\n");
                (*count) += 1;
                if((*count) >= 10){
                    *count = 0;
                    button_pressed = false;
                    fired = false;
                    gpio_put(PICO_DEFAULT_LED_PIN, 0);
                }
            }
            *last_message_time = now;
        }
}
