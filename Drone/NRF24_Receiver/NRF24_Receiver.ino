//Include Libraries
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <string.h>

//create an RF24 object
RF24 radio(9, 8);  // CE, CSN

//address through which two modules communicate.
const byte address_rx[6] = "00001";
const byte address_tx[6] = "00002";
uint16_t pwm_send;

void setup()
{
  while (!Serial);
  Serial.begin(9600);
  radio.begin();
  radio.setAutoAck(1);                    // Ensure autoACK is enabled
  radio.enableAckPayload();               // Allow optional ack payloads
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(3, OUTPUT);
  pinMode(10, OUTPUT);
  digitalWrite(3, 0);
  digitalWrite(5, 0);
  digitalWrite(6, 0);
  digitalWrite(10, 0);
}

void loop()
{
  //Read the data if available in buffer
  receive();
  if (radio.available())
  {
    char text[4] = {0};
    radio.read(&text, sizeof(text));
    String st = String(text);
    if(text!=""){
      String yes = "OK";
      transmit(yes, 2);
      Serial.println(st);
    }
    if(st.indexOf("U") != -1){
      uint8_t pwm = st.substring(st.indexOf("")+1, st.lastIndexOf("\n")).toInt(); 
      pwm_send = pwm;
      potencia_motors(pwm_send, 0);
    }
    if(st.indexOf("W") != -1){
      uint8_t pwm = st.substring(st.indexOf("")+1, st.lastIndexOf("\n")).toInt(); 
      pwm_send = pwm;
      potencia_motors(pwm_send, 1);
    }
    if(st.indexOf("S") != -1){
      uint8_t pwm = st.substring(st.indexOf("")+1, st.lastIndexOf("\n")).toInt(); 
      pwm_send = pwm;
      potencia_motors(pwm_send, 2);
    }
    if(st.indexOf("A") != -1){
      uint8_t pwm = st.substring(st.indexOf("")+1, st.lastIndexOf("\n")).toInt(); 
      pwm_send = pwm;
      potencia_motors(pwm_send, 3);
    }
    if(st.indexOf("D") != -1){
      uint8_t pwm = st.substring(st.indexOf("")+1, st.lastIndexOf("\n")).toInt(); 
      pwm_send = pwm;
      potencia_motors(pwm_send, 4);
    }
  }
}

void transmit(String enviar, uint8_t tm){
  wait_to_transmit(40);
  char send[tm]; 
  enviar.toCharArray(send, enviar.length()+1);
  radio.openWritingPipe(address_tx);
  radio.stopListening();
  radio.write(&send, sizeof(send));
  receive();
}

void receive(){
    //set the address
    radio.openReadingPipe(1, address_rx);
    //Set module as receiver
    radio.startListening();
}

void wait_to_transmit(uint16_t num){
    uint64_t timer1 = millis();
    while((timer1 + num) > millis() ){

    }
}

void potencia_motors(uint8_t pwm, int8_t mode){
  if(pwm <50)
    mode = 10;
  uint16_t pwm_add = pwm + 10;
  uint16_t pwm_sub = pwm - 50;
  if(pwm_add>255)
    pwm_add = 255;
  if(pwm_sub<0)
    pwm_sub = 0;
  uint32_t timer = millis();
  while((timer + 100)>millis()){
    switch (mode){
      case 0:
        analogWrite(10, pwm);
        analogWrite(6, pwm);
        analogWrite(3, pwm);
        analogWrite(5, pwm);
        break;
      case 1:
        analogWrite(10, pwm_sub);  //B_forward 10
        analogWrite(3, pwm_sub);   //A_forward 3
        analogWrite(6, pwm_add);   //B_back 6
        analogWrite(5, pwm_add);   //A_back 5
        break;
      case 2:
        analogWrite(10, pwm_add);  //B_forward
        analogWrite(3, pwm_add);   //A_forward
        analogWrite(6, pwm_sub);   //B_back
        analogWrite(5, pwm_sub);   //A_back
        break;
      case 3:
        analogWrite(10, pwm_add);  //B_forward
        analogWrite(3, pwm_sub);   //A_forward
        analogWrite(6, pwm_sub);   //B_back
        analogWrite(5, pwm_add);   //A_back
        break;
      case 4:
        analogWrite(10, pwm_sub);  //B_forward
        analogWrite(3, pwm_add);   //A_forward
        analogWrite(6, pwm_add);   //B_back
        analogWrite(5, pwm_sub);   //A_back
        break;
      default:
        release();
        break;
    }
  }
}
      
void release(){
    analogWrite(10, 0);
    analogWrite(6, 0);
    analogWrite(3, 0);
    analogWrite(5, 0);
}



