
/**
 * @file Pwm_generator.ino
 *
 * @mainpage Generador de señal PWM con duty y frecuencia variable
 *
 * @section Programa que recibe por medio de la terminal comandos 
 * para hacer generar una señal PWM de uno de los pin de  salida arduino 
 * pudiendo modificar los valores de .
 *
 * @section notes Notes
 * El programa solo tiene permitido 5 comandos con los cuales
 *  el usuario puede interactuar con la terminal.
 * Estos son  start, stop, duty, freq, reset para modificar
 * o hacer uso del generador.
 *
 * @section author Author
 * - Juan David Rios Rivera
 * - Mario Alejandro Tabares
 */
#include <TimerOne.h>

#define PWM 9 //Pin 5         //Pin 5 /**< Define el pin 5 a la señal salida del arduino */
//Variables flotantes
unsigned long freq = 1000;            /**< Define la frecuencia por defecto f=1000 */
unsigned long duty = 512;             /**< Define el duty por defecto d=512*/
//variables booleanas
bool activate;                        /**< Bandera que indica si esta activo o no el sistema */


void setup() {
      /**
  Primero se inicializa la frecuencia en el contador del timer, seguido de mandarle los valores 
  de entrada a la uncion que determina el pin de salida del PWM y el duty de la señal.
  se inicializan los budios de comunicacion y por ultimo se imprime algo en terminal para esperar 
  respuesta del usuario
  @param Ninguno función no recibe parámetros
  @return Esta función no retorna nada
  */
    Timer1.initialize(1000000/freq);
    Timer1.setPwmDuty(PWM, duty);
    Serial.begin(9600);
    Serial.print(">>");
}

void loop() {

  /**
    Función que se ejecuta en loop infinito en el arduino, aqui el loop se pregunta constantemente que comandos ingresa el usuario,
    Existen condicionales que determinan el comando ingresado por el usuario y debido a eso almacena o realiza
    cambion de la señal de salida.
  */
  
  if (Serial.available()>0){          
   /**if(Serial. acailable)
    Verifica si la terminal esta activa y a la espera de algun comando
  */
    String command = Serial.readStringUntil('\n');    /*! Pide el comando de la terminal */
    command.toUpperCase();                            /*! define siempre mayuscula el comando del usuario */
    if(command=="START"){
      /**if(comand=="START")
        Verifica si el comando es start, y activa la bandera activate para generar la salida de la señal
        por el pin del arduino 9, por ultimo muestra un mensage en pantalla indicando su activacion  
      */
        activate = 1;
        Serial.print("SIG. ACTIVATED");
        }
    if(command=="STOP"){
       /**if(comand=="STOP")
        Verifica si el comando es stop, y desactiva la bandera activate para generar la salida nula de la señal
        por el pin del arduino 9, por ultimo muestra un mensage en pantalla indicando su descativación   
      */
        activate = 0;
        Timer1.disablePwm(PWM);
        Serial.print("SIG. DISABLED");
    }
    if(command.indexOf("FREQ") != -1){
       /**if(comand.indexOf("FREQ != -1 "))
        Verifica si el comando de FREQ es escrito por la terminal 
        con algun valor que lo acompañe despues de la tecla espacio, 
        guarda el valor ingresado en la terminal en a y se lo asigna en frecuencia
        Se modifica la frecuencia de la señal actual, debido a que se inicializa 
        con el valor de la nueva frecuencia, imprime un mensage en terminal e imprime
        la frecuencia
      */
        unsigned long a = command.substring(command.indexOf(" ")+1, command.lastIndexOf("\n")).toInt(); 
        freq = a;
        Timer1.initialize(1000000/freq);
        Serial.print("DONE ");
        Serial.print(freq);
    }
    if(command.indexOf("DUTY") != -1){
      /**if(comand.indexOf("DUTY != -1 "))
        Verifica si el comando de DUTY es escrito por la terminal 
        con algun valor que lo acompañe despues de la tecla espacio, 
        guarda el valor ingresado en la terminal en d y se lo asigna en duty
        por medio de una matiz de transformacion de los valores del duty con la funcion map
        Se modifica el duty de la señal actual, debido a que se cambia el valor de pwm 
        con el valor del nuevo duty,  por ultimo imprime un mensage en terminal.
      */
      unsigned long d = command.substring(command.indexOf(" ")+1, command.lastIndexOf("\n")).toInt(); 
      duty = map(d, 0, 100, 0, 1023);
      Timer1.pwm(PWM, duty);
      Serial.print("DONE");
    }
    if(command=="RESET"){
      /**if(comand=="RESET"))
       declara la frecuencia y el duty por defecto.
      */
        Timer1.initialize(1000);
        duty = 512;
    }
    Serial.print('\n');   /*! Salto de linea */
    Serial.print(">>");   /*! imprime >> en terminal */
  }

  if(activate){
       /**if(activate))
      Si la bandera se encuentra activa, lanza la funcion de pwm con los valores del pin
      de salida del PWM y el duty que contenga actualmente.
      */
        Timer1.pwm(PWM, duty);
  }
  
}
