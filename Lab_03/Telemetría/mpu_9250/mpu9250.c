/** 
   @file gps.c
   @brief Example code to talk to a MPU9250 MEMS accelerometer and gyroscope.
   NOTE: Ensure the device is capable of being driven at 3.3v NOT 5v. The Pico
   GPIO (and therefor SPI) cannot be used at 5v.

   You will need to use a level shifter on the I2C lines if you want to run the
   board at 5v.

   Connections on Raspberry Pi Pico board and a generic MPU9250 board, other
   boards may vary.

   GPIO 4 (pin 6) MISO/spi0_rx-> ADO on MPU9250 board
   GPIO 5 (pin 7) Chip select -> NCS on MPU9250 board
   GPIO 6 (pin 9) SCK/spi0_sclk -> SCL on MPU9250 board
   GPIO 7 (pin 10) MOSI/spi0_tx -> SDA on MPU9250 board
   3.3v (pin 36) -> VCC on MPU9250 board
   GND (pin 38)  -> GND on MPU9250 board

   Note: SPI devices can have a number of different naming schemes for pins. See
   the Wikipedia page at https://en.wikipedia.org/wiki/Serial_Peripheral_Interface
   for variations.
   The particular device used here uses the same pins for I2C and SPI, hence the
   using of I2C names
*/

#include <stdio.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include <math.h>
#include "mpu9250.h"

#define PIN_MISO 4 //SAO
#define PIN_CS 5
#define PIN_SCK 6
#define PIN_MOSI 7 //SDA

#define SPI_PORT spi0
#define READ_BIT 0x80

void cs_select()
{
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 0); // Active low
    asm volatile("nop \n nop \n nop");
}

void cs_deselect()
{
    asm volatile("nop \n nop \n nop");
    gpio_put(PIN_CS, 1);
    asm volatile("nop \n nop \n nop");
}

/*************************************************************************************************/
/**
  @brief Resetea la IMU

  @returns Nothing
*/
void mpu9250_reset()
{
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    cs_select();
    spi_write_blocking(SPI_PORT, buf, 2);
    cs_deselect();
}

/*************************************************************************************************/
/**
  @brief Lee registros de la IMU
  @param reg Registro a leer
    @param buf Buffer donde se guardan los datos leidos
    @param len Longitud del buffer
  @returns Nothing
*/

void read_registers(uint8_t reg, uint8_t *buf, uint16_t len)
{
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    reg |= READ_BIT;
    cs_select();
    spi_write_blocking(SPI_PORT, &reg, 1);
    sleep_ms(10);
    spi_read_blocking(SPI_PORT, 0, buf, len);
    cs_deselect();
    sleep_ms(10);
}

/*************************************************************************************************/
/**
  @brief Lee el valor de aceleracion en los 3 ejes
    @param accel Buffer donde se guardan los datos leidos

  @returns Nothing
*/
void mpu9250_read_raw_accel(int16_t accel[3]) { //Used to get the raw acceleration values from the mpu
    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    read_registers(0x3B, buffer, 6);

    for (int i = 0; i < 3; i++) {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }
}

/*************************************************************************************************/
/**
  @brief Lee el valor de velocidad angular en los 3 ejes
    @param gyro Buffer donde se guardan los datos leidos

  @returns Nothing
*/
void mpu9250_read_raw_gyro(int16_t gyro[3]) {  //Used to get the raw gyro values from the mpu
    uint8_t buffer[6];
    
    read_registers(0x43, buffer, 6);

    for (int i = 0; i < 3; i++) {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }
}

/*************************************************************************************************/
/**
  @brief Lee el valor de campo magnetico en los 3 ejes
    @param magne Buffer donde se guardan los datos leidos

  @returns Nothing
*/
void mpu9250_read_raw_magnetometer(int16_t magne[3]){
    uint8_t buffer[6];

    read_registers(0x0C, buffer, 6); //cambiar por 0x4A si funciona

    for (int i = 0; i < 3; i++) {
        magne[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }
}

/*************************************************************************************************/
/**
  @brief Calibra los sensores, se debe mantener la IMU quieta
    @param sensor_data Buffer donde se guardan los datos leidos
    @param loop Numero de veces que se lee el sensor
    @param sensor Sensor a calibrar , 1, 2, 3 para acelerometro, giroscopio y magnetometro respectivamente

  @returns Nothing
*/
void calibrate(int16_t sensor_data[3], int loop, int sensor)  //Used to calibrate the gyro. The gyro must be still while calibration happens
{
    int16_t temp[3];
    for (int i = 0; i < loop; i++)
    {   
        switch (sensor)
        {
        case 1:
            mpu9250_read_raw_accel(temp);
            break;
        case 2:
            mpu9250_read_raw_gyro(temp);
            break;
        case 3:
            mpu9250_read_raw_magnetometer(temp);
            break;
        default:
            break;
        }
        sensor_data[0] += temp[0];
        sensor_data[1] += temp[1];
        sensor_data[2] += temp[2];
    }
    sensor_data[0] /= loop;
    sensor_data[1] /= loop;
    sensor_data[2] /= loop;
}

/*************************************************************************************************/
/**
  @brief Calcula los angulos de inclinacion a partir de los datos del acelerometro
    @param eulerAngles Buffer donde se guardan los angulos calculados
    @param accel Buffer donde estan los datos leidos acelerometro
    @param gyro Buffer donde estan los datos leidos giroscopo
    @param usSinceLastReading Tiempo transcurrido desde la ultima lectura

    
  @returns Nothing
*/
void calculate_angles(int16_t eulerAngles[2], int16_t accel[3], int16_t gyro[3], uint64_t usSinceLastReading) //Calculates angles based on the accelerometer and gyroscope. Requires usSinceLastReading to use the gyro.
{
    long hertz = 1000000/usSinceLastReading;
    
    if (hertz < 200)
    {
        calculate_angles_from_accel(eulerAngles, accel);
        return;
    }

    long temp = 1.l/(hertz * 65.5l);  

    eulerAngles[0] += gyro[0] * temp;
    eulerAngles[1] += gyro[1] * temp;

    eulerAngles[0] += eulerAngles[1] * sin(gyro[2] * temp * 0.1f);
    eulerAngles[1] -= eulerAngles[0] * sin(gyro[2] * temp * 0.1f);

    int16_t accelEuler[2];
    calculate_angles_from_accel(accelEuler, accel);

    eulerAngles[0] = eulerAngles[0] * 0.9996 + accelEuler[0] * 0.0004;
    eulerAngles[1] = eulerAngles[1] * 0.9996 + accelEuler[1] * 0.0004;
}

/*************************************************************************************************/
/**
  @brief Calcula los angulos del acelerometro
    @param eulerAngles Buffer donde se guardan los angulos calculados
    @param accel Buffer donde estan los datos leidos acelerometro
    
  @returns Nothing
*/
void calculate_angles_from_accel(int16_t eulerAngles[2], int16_t accel[3]) //Uses just the direction gravity is pulling to calculate angles.
{
    float accTotalVector = sqrt((accel[0] * accel[0]) + (accel[1] * accel[1]) + (accel[2] * accel[2]));

    float anglePitchAcc = asin(accel[1] / accTotalVector) * 57.296;
    float angleRollAcc = asin(accel[0] / accTotalVector) * -57.296;

    eulerAngles[0] = anglePitchAcc;
    eulerAngles[1] = angleRollAcc;
}

/*************************************************************************************************/
/**
  @brief Convierte los angulos de -90/90 a 360 usando la direccion de la gravedad
    @param eulerAngles Buffer donde se guardan los angulos calculados
    @param accel Buffer donde estan los datos leidos acelerometro
    @param fullAngles Buffer donde se guardan los angulos calculados en 360
    
  @returns Nothing
*/
void convert_to_full(int16_t eulerAngles[2], int16_t accel[3], int16_t fullAngles[2]) //Converts from -90/90 to 360 using the direction gravity is pulling
{
    if (accel[1] > 0 && accel[2] > 0) fullAngles[0] = eulerAngles[0];
    if (accel[1] > 0 && accel[2] < 0) fullAngles[0] = 180 - eulerAngles[0];
    if (accel[1] < 0 && accel[2] < 0) fullAngles[0] = 180 - eulerAngles[0];
    if (accel[1] < 0 && accel[2] > 0) fullAngles[0] = 360 + eulerAngles[0];

    if (accel[0] < 0 && accel[2] > 0) fullAngles[1] = eulerAngles[1];
    if (accel[0] < 0 && accel[2] < 0) fullAngles[1] = 180 - eulerAngles[1];
    if (accel[0] > 0 && accel[2] < 0) fullAngles[1] = 180 - eulerAngles[1];
    if (accel[0] > 0 && accel[2] > 0) fullAngles[1] = 360 + eulerAngles[1];
}

/*************************************************************************************************/
/**
  @brief Inicializa el mpu9250 a través del SPI
    @param Nothing
    
  @returns Nothing
*/
void start_spi() //Starts the mpu and resets it
{
    spi_init(SPI_PORT, 500 * 1000);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // Chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    mpu9250_reset();

        // See if SPI is working - interrograte the device for its I2C ID number, should be 0x71
    uint8_t id;
    read_registers(0x75, &id, 1);
    printf("I2C address is 0x%x\n", id);
}