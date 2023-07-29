/*
 * Copyright (c) 2021 Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 * 
 * 
 * This example uses ABP to join the LoRaWAN network and then sends a
 * "hello world" uplink message periodically and prints out the
 * contents of any downlink message.
 */

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "pico/lorawan.h"
#include "tusb.h"

// edit with LoRaWAN Node Region and ABP settings 
#include "config.h"


// pin configuration for SX1276 radio module
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

// ABP settings
const struct lorawan_abp_settings abp_settings = {
    .device_address = LORAWAN_DEV_ADDR,
    .network_session_key = LORAWAN_NETWORK_SESSION_KEY,
    .app_session_key = LORAWAN_APP_SESSION_KEY,
    .channel_mask = LORAWAN_CHANNEL_MASK
};

// variables for receiving data
int receive_length = 0;
uint8_t receive_buffer[242];
uint8_t receive_port = 0;

int main( void )
{
    // initialize stdio and wait for USB CDC connect
    stdio_init_all();

    /*while (!tud_cdc_connected()) {
        tight_loop_contents();
    }*/
    
    printf("Pico LoRaWAN - Hello ABP\n\n");

    // uncomment next line to enable debug
    // lorawan_debug(true);

    // initialize the LoRaWAN stack
    printf("Initilizating LoRaWAN ... ");
    if (lorawan_init_abp(&sx12xx_settings, LORAWAN_REGION, &abp_settings) < 0) {
        printf("failed!!!\n");
        while (1) {
            tight_loop_contents();
        }
    } else {
        printf("success!\n");
    }

    // Start the join process and wait
    printf("Joining LoRaWAN network ... ");
    lorawan_join();

    while (!lorawan_is_joined()) {
        lorawan_process();
    }
    printf("joined successfully!\n");
    

    uint32_t last_message_time = 0;
    uint8_t counter = 0;
    uint8_t adr = 5;
    changeADR(adr);
    // loop forever
    float latitude = 6.263448;
    float longitude =  -75.561798;
    while (1) {
        // let the lorwan library process pending events
        
        lorawan_process();
        // get the current time and see if 5 seconds have passed
        // since the last message was sent
        uint32_t now = to_ms_since_boot(get_absolute_time());
        
        if ((now - last_message_time) > 3000) {
            //char message[18];
            //snprintf(message, sizeof(message), "hello world! %d", adr);
            uint8_t len = snprintf(NULL,0, "1, %f, %f, %d", latitude, longitude, getADR());
            char message[len];
            sprintf(message, "1, %f, %f, %d", latitude, longitude, getADR());

            //Mayor distancia ADR menor, el ADR sube si bajo SF, es decir largas distancia usar SF alto
            //ADR 3 SF 9 BW 125 cortas distancias
            //ADR 4 SF 10 BW 125 
            //ADR 5 SF 7 BW 125 Largas distancias
            //ADR 6 SF 8 BW 500 cortas distancias


            // try to send an unconfirmed uplink message
            printf("sending unconfirmed message '%s' ... ADR: %d Port: %d ", message, getADR(), 6);
            if (lorawan_send_unconfirmed(message, strlen(message), 6) < 0) {
                printf("failed!!!\n");
                counter++;
            } else {
                printf("success!\n");
                counter++;
                
            }
            if(counter>20){
                counter = 0;
                adr++;
                changeADR(6);
            }

            last_message_time = now;
        }

        // check if a downlink message was received
        receive_length = lorawan_receive(receive_buffer, sizeof(receive_buffer), &receive_port);
        if (receive_length > -1) {
            printf("received a %d byte message on port %d: ", receive_length, receive_port);

            for (int i = 0; i < receive_length; i++) {
                printf("%02x", receive_buffer[i]);
            }
            printf("\n");
        }
    }

    return 0;
}