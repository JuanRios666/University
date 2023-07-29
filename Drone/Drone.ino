#include <SoftwareSerial.h>
//SoftwareSerial Serial (7, 8); //RX, TX
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <PID_v1.h>
#include <EEPROM.h>
//#include "MPU6050.h" // not necessary if using MotionApps include file

//Include Libraries
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <string.h>


#define ADDR_PITCH_OFFSET 0
#define ADDR_ROLL_OFFSET 1
#define FL_MOTOR 10
#define FR_MOTOR 3
#define BR_MOTOR 6
#define BL_MOTOR 5
/*
      /B_forward_left 10
      /A_forward_right 3
      /B_back_right 6
      /A_back_left 5
  */

// -------------------------------RF24-----------------------------------
//create an RF24 object
RF24 radio(9, 8);  // CE, CSN

//address through which two modules communicate.
const byte address_rx[6] = "00001";
const byte address_tx[6] = "00002";
uint16_t pwm_send;

//---------------------------------PID------------------------------------
//Define Variables we'll be connecting to
double rollSetpoint, rollInput, rollOutput;
double pitchSetpoint, pitchInput, pitchOutput;

//Define the aggressive and conservative Tuning Parameters
double consKp = 5, consKi = 10, consKd = 0.05;

PID pitchPID(&rollInput, &rollOutput, &rollSetpoint, consKp, consKi, consKd, DIRECT);
PID rollPID(&pitchInput, &pitchOutput, &pitchSetpoint, consKp, consKi, consKd, DIRECT);
//------------------------------------------------------------------------


// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define LED_PIN 2 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float ypr_cal[3];

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


#define ALPHA 0.1
#define MULTIPLIER 6.67
float motorBattery;

int targetSpeed[4];

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
  Serial.begin(9600);
  while (!Serial);
  radio.begin();
  radio.setAutoAck(1);                    // Ensure autoACK is enabled
  radio.enableAckPayload();               // Allow optional ack payloads

  for (int i = 0; i < 3; i++) {
    ypr_cal[i] = 0.0;
  }

  //Enable internal reference of 1.1V
  //initialise battery level array with current battery level value
  pinMode(A0, INPUT);
  analogReference(INTERNAL);
//  float tmp = analogRead(A0) / 1023.0 * MULTIPLIER;
  motorBattery = 4.2;
  //------------------------------PID----------------------------------
  //initialize the variables we're linked to
  pitchInput = 0.0;
  rollInput = 0.0;

  pitchSetpoint = 0.0;
  rollSetpoint = 0.0;

  //turn the PID on
  pitchPID.SetMode(AUTOMATIC);
  rollPID.SetMode(AUTOMATIC);

  pitchPID.SetOutputLimits(-20, 20);
  rollPID.SetOutputLimits(-20, 20);
  //-------------------------------------------------------------------
  for (int i = 0; i < 4; i++) {
    targetSpeed[i] = 0;
  }

  pinMode(FL_MOTOR, OUTPUT);
  pinMode(FR_MOTOR, OUTPUT);
  pinMode(BR_MOTOR, OUTPUT);
  pinMode(BL_MOTOR, OUTPUT);

  digitalWrite(FL_MOTOR, 0);
  digitalWrite(FR_MOTOR, 0);
  digitalWrite(BR_MOTOR, 0);
  digitalWrite(BL_MOTOR, 0);

  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize Serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(9600);
  //while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  //while (Serial.available() && Serial.read()); // empty buffer
  //while (!Serial.available());                 // wait for data
  //while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(131);
  mpu.setYGyroOffset(131);
  mpu.setZGyroOffset(131);
  mpu.setZAccelOffset(16384); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
int myReading = 0;
void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  receive_rf();
  if (radio.available()) {
    char text[4] = {0};
    radio.read(&text, sizeof(text));
    String st = String(text);
    if(text!=""){
      String yes = "OK";
      transmit(yes, 2);
      Serial.println(st);
    }
    if(st.indexOf("U") != -1){  //Solo con U asigno directamente el pwm a los motores
      uint8_t pwm = st.substring(st.indexOf("")+1, st.lastIndexOf("\n")).toInt(); 
      pwm_send = pwm;
      potencia_motors(pwm_send, 0, 0);
    }
    if(st.indexOf("W") != -1){
      uint8_t pwm = st.substring(st.indexOf("")+1, st.lastIndexOf("\n")).toInt(); 
      uint8_t factor = pwm;
      potencia_motors(pwm_send, 1, factor);
    }
    if(st.indexOf("S") != -1){
      uint8_t pwm = st.substring(st.indexOf("")+1, st.lastIndexOf("\n")).toInt(); 
      uint8_t factor = pwm;
      potencia_motors(pwm_send, 2, factor);
    }
    if(st.indexOf("A") != -1){
      uint8_t pwm = st.substring(st.indexOf("")+1, st.lastIndexOf("\n")).toInt(); 
      uint8_t factor = pwm;
      potencia_motors(pwm_send, 3, factor);
    }
    if(st.indexOf("D") != -1){
      uint8_t pwm = st.substring(st.indexOf("")+1, st.lastIndexOf("\n")).toInt(); 
      uint8_t factor = pwm;
      potencia_motors(pwm_send, 4, factor);
    }
    myReading = pwm_send; //Aqui al parecer se asigna de nuevo el mismo pwm a los motores pero es un trick para que se vuelva a estar en el punto de estabilizacion
    for (int i = 0; i < 4; i++) {
      targetSpeed[i] = myReading;
    }

    runIndividual(targetSpeed);
    while (Serial.available())  //flushing anything that wasn't read
      Serial.read();
    //      runIndividual(myReading);
    //      checkMotor(myReading);
    //        checkIndividual(myReading);
  }

  // wait for MPU interrupt or extra packet(s) available
  // while (!mpuInterrupt && fifoCount < packetSize) {
  // if you are really paranoid you can frequently test in between other
  // stuff to see if mpuInterrupt is true, and if so, "break;" from the
  // while() loop to immediately process the MPU data
  //    }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;


    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    //----------------------------------------PID-----------------------------------------
    if (myReading == 0) {
      Serial.println(F("CALIBRATING"));
      ypr_cal[0] = ypr[0] * 180 / M_PI;
      ypr_cal[1] = ypr[1] * 180 / M_PI;
      ypr_cal[2] = ypr[2] * 180 / M_PI;

      //              ypr_cal[1] = 1.26;
      //              ypr_cal[2] = -0.55;
    }

    pitchInput = ypr[1] * 180 / M_PI - ypr_cal[1];
    rollInput = ypr[2] * 180 / M_PI - ypr_cal[2];

    //            pitchInput /= 2.0;
    //            rollInput /= 2.0;

    pitchPID.Compute();
    rollPID.Compute();

    int actSpeed[4];
    stabilise (targetSpeed, actSpeed, rollOutput, pitchOutput);
//    targetSpeed = actSpeed; // should this behere or not?

    Serial.print(F("pitchInput="));
    Serial.print(pitchInput);
    Serial.print(F("   pitchOutput="));
    Serial.print(pitchOutput);

    Serial.print(F("   rollInput="));
    Serial.print(rollInput);
    Serial.print(F("   rollOutput="));
    Serial.print(rollOutput);

    Serial.print(F("   mot[0]="));
    Serial.print(actSpeed[0]);
    Serial.print(F("   mot[1]="));
    Serial.print(actSpeed[1]);
    Serial.print(F("   mot[2]="));
    Serial.print(actSpeed[2]);
    Serial.print(F("   mot[3]="));
    Serial.println(actSpeed[3]);

    //runIndividual (actSpeed);
    //            checkIndividual(myReading, actSpeed);
    //------------------------------------------------------------------------------------
    motorBattery = smoothBattery(motorBattery, analogRead(A0) / 1023.0 * MULTIPLIER, ALPHA);
    if (motorBattery < 2.0){
      Serial.println (F("WARNING! LOW BATTERY!"));
    }
    Serial.print(motorBattery);
    Serial.print(F("   ypr   "));
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("   ");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("   ");
    Serial.println(ypr[2] * 180 / M_PI);


    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
  delay(100);
}

float smoothBattery (float prevEntry, float newEntry, float alpha) {
  return (1-alpha) * prevEntry + alpha * newEntry;
}

void setSpeed(int val) {
  analogWrite(FL_MOTOR, val);
  analogWrite(FR_MOTOR, val);
  analogWrite(BR_MOTOR, val);
  analogWrite(BL_MOTOR, val);
}

void stabilise (int* currSpeed, int* actSpeed, float rollDiff, float pitchDiff) {
  actSpeed[0] = (int) currSpeed[0] + (rollDiff) - (pitchDiff);  //each motor has actual Speed and speed at which we want them to fly...
  actSpeed[1] = (int) currSpeed[1] + (rollDiff) + (pitchDiff);
  actSpeed[2] = (int) currSpeed[2] - (rollDiff) + (pitchDiff);  //actual Speed is calculated as follows +- half rollDiff +- half pitchDiff
  actSpeed[3] = (int) currSpeed[3] - (rollDiff) - (pitchDiff);

  for (int i = 0; i < 4; i ++) {
    if (actSpeed[i] < 0 )
      actSpeed[i] = 0;
  }
}

void checkIndividual (int motor, int* actSpeed) {
  analogWrite(FL_MOTOR, 0);
  analogWrite(FR_MOTOR, 0);
  analogWrite(BR_MOTOR, 0);
  analogWrite(BL_MOTOR, 0);

  if (motor == 0)
    analogWrite(FL_MOTOR, actSpeed[0]);
  if (motor == 1)
    analogWrite(FR_MOTOR, actSpeed[1]);
  if (motor == 2)
    analogWrite(BR_MOTOR, actSpeed[2]);
  if (motor == 3)
    analogWrite(BL_MOTOR, actSpeed[3]);
}

void runIndividual (int* actSpeed) {
  analogWrite(FL_MOTOR, actSpeed[0]);
  analogWrite(FR_MOTOR, actSpeed[1]);
  analogWrite(BR_MOTOR, actSpeed[2]);
  analogWrite(BL_MOTOR, actSpeed[3]);
}

void transmit(String enviar, uint8_t tm){
  wait_to_transmit(40);
  char send[tm]; 
  enviar.toCharArray(send, enviar.length()+1);
  radio.openWritingPipe(address_tx);
  radio.stopListening();
  radio.write(&send, sizeof(send));
  receive_rf();
}

void receive_rf(){
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

void potencia_motors(uint8_t pwm, int8_t mode, uint8_t factor){
  if(pwm <50)
    mode = 10;
  uint16_t pwm_add = pwm + factor;
  uint16_t pwm_sub = pwm - factor;
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
