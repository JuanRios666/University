#include <string.h>
#include <stdint.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU9250.h>
#include <SoftwareSerial.h>

SoftwareSerial gpsSerial(0, 1);  // RX, TX
MPU9250 mpu;  // Crear una instancia del objeto MPU9250

String frameGPS = "";
float latitude = 0;   //  Latitud del modulo GPS
float longitude = 0;  //  Longitud del modulo GPS
int16_t ax, ay, az;   // Variables para almacenar los datos del acelerómetro
int16_t gx, gy, gz;   // Variables para almacenar los datos del giroscopio
uint32_t time_old = 0;

void setup() {
  Wire.begin();  // Iniciar la comunicación I2C
  Serial.begin(115200);  // Iniciar la comunicación serial a 115200 bps
  mpu.initialize();  // Inicializar el MPU9250
  gpsSerial.begin(9600);  // Inicializa la comunicación serial con el módulo GPS
  if (mpu.testConnection())
    Serial.println("MPU9250 inicializado correctamente.");
  delay(2000);
}

void loop() {
  //readGPSL76X();
  readMPU9250();
  sendData();
  //toDoControl();
}

void readGPSL76X() {
  if (gpsSerial.available()) {
    char data = gpsSerial.read();
    frameGPS += data;
    if (data == '\n') {
      filterGPSMessage(frameGPS);
      frameGPS = "";
      Serial.print("Latitude=");
      Serial.print(latitude);
      Serial.print(", Longitude=");
      Serial.println(longitude);
      latitude = 0;
      longitude = 0;
    }
  }
}

void filterGPSMessage(String message) {
  if (message.startsWith("$GNRMC")) {
    getLatitudeLongitude(message);
  }
}

void getLatitudeLongitude(String trama) {
  char* campo;
  char* tramaCopia = strdup(trama.c_str());
  
  campo = strtok(tramaCopia, ",");
  int campoIndex = 0;
  while (campo != NULL) {
    if (campoIndex == 3) {
      float lat = atof(campo);
      int aux = (int)lat/100;
      latitude = (lat-(100*aux))/60 + (float)aux;
    } else if (campoIndex == 5) {
      float lon = atof(campo);
      int aux2 = (int)lon/100;
      longitude = (lon-(100*aux2))/60 + (float)aux2;
      break;
    }
    campo = strtok(NULL, ",");
    campoIndex++;
  }
  free(tramaCopia);
}

void readMPU9250() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);  // Obtener datos del acelerómetro y giroscopio
}

void sendData() {
  if (millis() - time_old > 1000) {
    showData();
    time_old = millis();
  }
}

void showData() {
  Serial.print("Latitude=");
  Serial.print(latitude, 6);
  Serial.print("    Longitude=");
  Serial.print(longitude, 6);
  Serial.print("    Ax=");
  Serial.print(ax / 16384.0);
  Serial.print("    Ay=");
  Serial.print(ay / 16384.0);
  Serial.print("    Az=");
  Serial.print(az / 16384.0);
  Serial.print("    Gx=");
  Serial.print(gx / 131.0);
  Serial.print("    Gy=");
  Serial.print(gy / 131.0);
  Serial.print("    Gz=");
  Serial.println(gz / 131.0);
}

void toDoControl() {
  Serial.println("Haciendo el control...");
}
