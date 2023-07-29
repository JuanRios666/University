/**
 @mainpage Programa que recupera la informacion de los sensores instaldos en un dron y los envia por telemetria usando wifi
 * 
 * @section Descripcion
 * Se recibe los datos de la imu, acelerometro, giroscopo,
 * magentometros, y lee los datos del pwm aplicado a cada motor, ademas de las 
 * coordenadas GPS de donde se encuentra en cada momento, para enviar los datos
 * como una trama separada por comas, a través de WIFI
 * 
 *
 * @section Notes
 * El sistema inicia conectandose a una wifi que se debe especificar en el 
 * driver del wifi, además de la direccion del servidor donde se da apertura al 
 * puerto para escuchar y capturar las tramas que despues son procesadas en
 * programas externos usando python.
 *
  @file main.c
  @brief Módulo principal del sistema

    Este módulo se encarga de inicializar los módulos de comunicación, y de leer   los datos de los sensores
    para enviarlos al servidor.

  @author Juan David Rios Rivera - Mario Alejandro Tabares Orjuela
  @date 07/05/2022

*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "mpu9250.h"
#include "gps.h"
#include "wifi.h"
#include "hardware/uart.h"
#include "hardware/gpio.h"
 
#define INT_PIN 10                 /**< Pin de la interrupción del GPS*/

volatile bool int_pressed = false; /**< Booleana que se activa cuando llena una interrupción*/

void gpio_int_handler();
void aplicar_calibracion(int16_t gyro[3], int16_t acceleration[3], int16_t magne[3],int16_t gyroCal[3],int16_t acceCal[3],int16_t magCal[3], float gyro_send[3], float acc_send[3], float mag_send[3]);

/*************************************************************************************************/
/**
  @brief Función principal donde se inicializa el sistema y se ejecuta el programa principal en un loop infinito
  Se inicializan los puertos UART y SPI, para comunicarse con el GPS, WIFI y la IMU. 
  Se inicializa el pin de la interrupción del GPS, y se configura la interrupción para que se ejecute la función
  Se calibran los sensores y se guardan los datos de calibración en variables globales
  Se conecta el WIFI a la red y se establece el modo de operación
  Se ejecuta un loop infinito donde se leen los datos de los sensores, se aplican las calibraciones y se envían al servidor
  las variables en un trama separada por comas, usando la comunicacion TCP/IP establecida en el WIFI  
  @returns entero
*/
int main()
{
    stdio_init_all();

    // Configure gps
    init_uart0();
    // Configura uart wifi
    init_uart1();
    //Establece interrupcion del gps
    gpio_init(INT_PIN);                      //inicio el boton de la interrupción
    gpio_set_dir(INT_PIN, GPIO_IN);          //seteo el boton como entrada
    gpio_pull_up(INT_PIN);                   //activo la resistencia de pull up

    // Configure INT_PIN for edge-triggered interrupts
    gpio_set_irq_enabled_with_callback(INT_PIN, GPIO_IRQ_EDGE_RISE, true, &gpio_int_handler);
    //uart_set_irq_enables(UART_ID, true, false); //habilito las interrupciones de la uart
    //irq_set_exclusive_handler(UART0_IRQ, uart_rx_handler); //seteo el handler de la interrupción
    
    //Connect to wifi 
    sleep_ms(2000);
    connectToWifi(); 
    sleep_ms(2000);
    
    printf("Hello, MPU9250! Reading raw data from registers via SPI...\n");
    start_spi();  //Starts the mpu

    int16_t acceleration[3], acceCal[3], gyro[3], gyroCal[3], eulerAngles[2], fullAngles[2], magne[3], magCal[3]; //Declares the variables required for calculations
    
    absolute_time_t timeOfLastCheck;

    calibrate(gyroCal, 100, 2);  //Calibrates the gyro
    calibrate(acceCal, 100, 1);  //Calibrates the accel
    calibrate(magCal, 100, 3);   //Calibrates the magnetometer
    printf("Gyro cal: %d, %d, %d\n", gyroCal[0], gyroCal[1], gyroCal[2]);
    printf("Accel cal: %d, %d, %d\n", acceCal[0], acceCal[1], acceCal[2]);
    printf("Magnetometer cal: %d, %d, %d\n", magCal[0], magCal[1], magCal[2]);

    /*mpu9250_read_raw_accel(acceleration);  //Sets the absolute angle using the direction of gravity
    calculate_angles_from_accel(eulerAngles, acceleration);
    timeOfLastCheck = get_absolute_time();*/

    float latitude = 0;
    float longitude = 0;
    float gyro_send[3], acc_send[3], mag_send[3];
    int16_t pwm = 20;

    while (1)
    {
        mpu9250_read_raw_accel(acceleration);  //Reads the accel and gyro 
        mpu9250_read_raw_gyro(gyro);
        mpu9250_read_raw_magnetometer(magne);
        
        aplicar_calibracion(gyro,acceleration,magne, gyroCal, acceCal, magCal, gyro_send, acc_send, mag_send);

        /*calculate_angles(eulerAngles, acceleration, gyro, absolute_time_diff_us(timeOfLastCheck, get_absolute_time()));  //Calculates the angles
        timeOfLastCheck = get_absolute_time();

        convert_to_full(eulerAngles, acceleration, fullAngles);

        printf("Acc. X = %f, Y = %f, Z = %f\n", acc_send[0], acc_send[1], acc_send[2]);  //Prints the angles
        printf("Gyro. X = %f, Y = %f, Z = %f\n", gyro_send[0], gyro_send[1], gyro_send[2]);
        printf("Mag. X = %f, Y = %f, Z = %f\n", mag_send[0], mag_send[1], mag_send[2]);
        printf("Euler. Roll = %d, Pitch = %d\n", eulerAngles[0], eulerAngles[1]);
        printf("Full. Roll = %d, Pitch = %d\n", fullAngles[0], fullAngles[1]);*/

        if(int_pressed){
            uart_rx_handler();
            int_pressed = false;
        }
        
        if (new_data_available &  (strncmp(sentence_buffer, "$GNRMC", 6) == 0)) {
            new_data_available = false;
            extract_lat_long(sentence_buffer, &latitude, &longitude);
            if(latitude >= 0.0F || longitude >= 0.0F){
                char data_acc[100];
                sprintf(data_acc, "%f,%f,%f", acc_send[0], acc_send[1], acc_send[2]);
                printf("The acc is: %s\n", data_acc);
                
                char data_gyro[100];
                sprintf(data_gyro, "%f,%f,%f", gyro_send[0], gyro_send[1], gyro_send[2]);
                printf("The gyro is: %s\n", data_gyro);

                char data_mag[100];
                sprintf(data_mag, "%f,%f,%f", mag_send[0], mag_send[1], mag_send[2]);
                printf("The mag is: %s\n", data_mag);

                char data_pwm[100];
                sniprintf(data_pwm, sizeof(data_pwm), "%i,%i,%i,%i", pwm+10, pwm+20, pwm+5, pwm);
                printf("The pwm is: %s\n", data_pwm);

                char data_gps[100];
                sprintf(data_gps, "%f,%f", latitude, longitude);
                printf("The gps is: %s\n", data_gps);

                char data[500];
                sniprintf(data, sizeof(data), "%s,%s,%s,%s,%s", data_acc, data_gyro, data_mag, data_pwm, data_gps);
                printf("Texto a enviar: %s\n", data);
                send_sensor_values(data);
            }
        }
    }
}

/*************************************************************************************************/
/**
  @brief Aplica la calibración a los valores de los sensores recuperados, 
        preparando los datos para ser enviados


    @param gyro Arreglo de 3 elementos donde están guardados los valores de los ejes del giroscopio
    @param acceleration Arreglo de 3 elementos donde están guardados los valores de los ejes de la aceleración
    @param magne Arreglo de 3 elementos donde están guardados los valores de los ejes del magnetómetro
    @param gyroCal Arreglo de 3 elementos donde están guardados los valores de calibración del giroscopio
    @param acceCal Arreglo de 3 elementos donde están guardados los valores de calibración de la aceleración
    @param magCal Arreglo de 3 elementos donde están guardados los valores de calibración del magnetómetro
    @param gyro_send Arreglo de 3 elementos donde se guardaran los valores de los ejes del giroscopio en grados por segundo, calibrados
    @param acc_send Arreglo de 3 elementos donde se guardaran los valores de los ejes de la aceleración en g's, calibrados  
    @param mag_send Arreglo de 3 elementos donde se guardaran los valores de los ejes del magnetómetro en Gauss, calibrados

 @returns Nothing

*/

void aplicar_calibracion(int16_t gyro[3], int16_t acceleration[3], int16_t magne[3],int16_t gyroCal[3],int16_t acceCal[3],int16_t magCal[3], float gyro_send[3], float acc_send[3], float mag_send[3]){

    for(int i=0; i<3; i++){
        gyro[i] -= gyroCal[i];  //Applies the calibration
        acceleration[i] -= acceCal[i];
        magne[i] -= magCal[i];
        gyro_send[i] = ((float)gyro[i]/32768)*250; //Converts to degrees per second
        acc_send[i] = ((float)acceleration[i]/32768)*2; //Converts to g's
        mag_send[i] = ((float)magne[i]/32768)*4800; //Converts to Gauss
    }

}

/*************************************************************************************************/
/**
  @brief Atiende la interrupcion del GPIO 10, el cual es el que se conecta al pin de interrupcion
  proveniente del pin TX del GPS

  @returns Nothing

*/
void gpio_int_handler() {
    int_pressed = true;
}