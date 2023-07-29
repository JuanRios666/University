//Include Libraries
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <string.h>
#define X_D A0
#define Y_D A1
#define Y_I A2
#define REFRESH 100

//create an RF24 object
RF24 radio(9, 8);  // CE, CSN

//address through which two modules communicate.
const byte address_tx[6] = "00001";
const byte address_rx[6] = "00002";
uint64_t timer1 = millis();
int16_t accel, roll, pitch;  // roll variable X y pitch variable Y 
int16_t accel_old, roll_old, pitch_old;  // roll variable X y pitch variable Y 
uint32_t deadline = 0;

void setup()
{
  Serial.begin(9600);
  pinMode(X_D, INPUT);
  pinMode(Y_D, INPUT);
  pinMode(Y_I, INPUT);
  radio.begin();
  radio.setAutoAck(1);                    // Ensure autoACK is enabled
  radio.enableAckPayload();               // Allow optional ack payloads
  deadline = millis();
}

void loop()
{   
    String msg_RC;
    control(&accel, &pitch, &roll);
    if((deadline + REFRESH) < millis()){
      if((accel_old > accel +2) || (accel_old < accel -2)){
        if( (accel < 135 || accel > 138)){ 
          msg_RC = crear_command(&accel, 0);
          accel_old = accel;
        }
      }else {
        accel_old = accel;
      }
      if((roll_old > roll +2) || (roll_old < roll -2)){
        msg_RC = crear_command(&roll, 1);
        roll_old = roll;
      }else{
        roll_old = roll;
      }
      if((pitch_old > pitch +2) || (pitch_old < pitch -2)){
        msg_RC = crear_command(&pitch, 2);
        pitch_old = pitch;
      }else{
        pitch_old = pitch;
      }
      if(msg_RC!= ""){
        Serial.println(msg_RC);
        transmit(msg_RC, 4);
      }
      /*if(Serial.available()){
          String command = Serial.readStringUntil('\n');
          transmit(command, 4);
      }*/
      deadline = millis();
    }

}

void transmit(String enviar, uint8_t tm){
  char send[tm]; 
  enviar.toCharArray(send, enviar.length()+1);
  radio.openWritingPipe(address_tx);
  radio.stopListening();
  radio.write(&send, sizeof(send));
  wait_response(200);
}

void wait_response(uint16_t num){
  radio.openReadingPipe(1, address_rx);
  //Set module as receiver
  radio.startListening();
  uint64_t timer2 = millis();
  char text[2] = {0};
  while((timer2 + num) > millis() ){
    while (radio.available())
      {
        radio.read(&text, sizeof(text));
        String st = String(text);
        Serial.println("Response: " + st);
      }
  }
}

 void wait_10ms(){
   uint64_t timer2 = millis();
   while((timer2+10) > millis() ){
   }
 }

void control(int16_t *accel, int16_t *x, int16_t *y){
    int16_t mov_x, mov_y, mov_accel;
    mov_x = analogRead(X_D);
    *x = map(mov_x, 0, 1023, -180, 185);
    wait_10ms();
    mov_y = analogRead(Y_D);
    *y = map(mov_y, 0, 1023, -180, 188);
    wait_10ms();
    mov_accel = analogRead(Y_I);
    *accel = map(mov_accel, 0, 1023, 0, 255);
}

String crear_command(int16_t *value, int8_t name){
    String comando;
    switch (name){
      case 0:
            comando = "U" + String(*value);
            break;
      case 1: // Aqui viene el muestreo del joystick derecho en el eje x , es decir el giro a la derecha o izquierda
          if(*value >= 0){
            int16_t pwm = map(*value, 0, 180, 0, 30); // Se hace un muestreo hasta 30 que es lo maximo que manda el giro
            comando = "D" + String(pwm);
          }else{
            int16_t pwm = map(*value, 0, -180, 0, 30);
            comando = "A" + String(pwm);
          }
          break;
      case 2:// Aqui viene el muestreo del joystick derecho en el eje y , es decir el cabaceo arriba o abajo
          if(*value >= 0){
            int16_t pwm = map(*value, 0, 180, 0, 30);   // Se hace un muestreo hasta 30 que es lo maximo que manda el cabeceo
            comando = "W" + String(pwm);
          }else{
            int16_t pwm = map(*value, 0, -180, 0, 30);
            comando = "S" + String(pwm);
          }
          break;
    }
    return comando;
}

