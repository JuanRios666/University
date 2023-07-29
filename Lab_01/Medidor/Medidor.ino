/**
 * @file Medidor.ino
 *
 * @mainpage Medidor de Frecuencia y multiplexado en 7 segmentos
 *
 * @section Programa que lee la frecuencia 
 * que entra en una onda cuadrada por uno de los pines del arduino 
 * y lo muestra en 5 display 7 segmentos.
 *
 * @section Circuit
 * - Salidas del arduino para controlar el encendido displays en orden A0, A1, A2, A3, A4
 * con resistencias de 1K a la base de los transistores, estos con el emisor a VCC y el colector a el pin de activacion de cada 7 segmmentos
 * - Salidas del arduino para encender los segmentos de ánodo común 5, 6, 7, 8, 9, 10, 11
 * con resistencias de 220 ohm a cada uno de los segmentos, todo estos unidos en serie unos con otros
 * para encenderse todos al mismo tiempo si se activaran de igual manera
 *
 * @section notes Notes
 * Se definen las variables para encender los displays 7 segmentos, 
 * un arreglo con los pines que se encienden en bajo, al ser ánodo común 
 * El orden de los led es : {a, b, c, d, e, f, g}
 * Igualmente se define como variable un arreglo para encender los displays en el orden
 * {display 1, display 2, display3, display 4, display 5}
 * El orden igualmente es {unidades, decenas, centenas, miles, diezmiles}
 * Los primeros dos displays se usan al encenderse con 1, por tener transitores NPN, los siguientes con 0 al tener PNP
 * finalmente el display 5 se enciende con 1 por tener un NPN
 *
 * @section author Author
 * - Juan David Rios Rivera
 * - Mario Alejandro Tabares
 */

//Variables Constantes Globales
const int cero[]= {0,0,0,0,0,0,1};    /**< Numero cero escrito en display 7 segmentos, ánodo comun */
const int one[] = {1,0,0,1,1,1,1};    /**< Numero uno escrito en display 7 segmentos, ánodo comun */
const int two[] = {0,0,1,0,0,1,0};    /**< Numero dos escrito en display 7 segmentos, ánodo comun */
const int three[] = {0,0,0,0,1,1,0};  /**< Numero tres escrito en display 7 segmentos, ánodo comun */
const int four[] = {1,0,0,1,1,0,0};   /**< Numero cuatro escrito en display 7 segmentos, ánodo comun */
const int five[] = {0,1,0,0,1,0,0};   /**< Numero cinco escrito en display 7 segmentos, ánodo comun */
const int six[] = {0,1,0,0,0,0,0};    /**< Numero seis escrito en display 7 segmentos, ánodo comun */
const int seven[] = {0,0,0,1,1,1,1};  /**< Numero siete escrito en display 7 segmentos, ánodo comun */
const int eight[]= {0,0,0,0,0,0,0};   /**< Numero ocho escrito en display 7 segmentos, ánodo comun */
const int nine[]= {0,0,0,0,1,0,0};    /**< Numero nueve escrito en display 7 segmentos, ánodo comun */
const int mayor[]= {0,1,1,1,1,1,1};   /**< Sólo el segmentos a activo en el display 7 segmentos, ánodo comun */
const int menor[] = {1,1,1,0,1,1,1};  /**< Sólo el segmentos d activo en el display 7 segmentos, ánodo comun */

const int pinA0[] = {1,0,1,1,0};      /**< Encender solo el primer display, con transistor NPN, las unidades*/
const int pinA1[] = {0,1,1,1,0};      /**< Encender solo el segundo display, con transistor NPN, las decenas */
const int pinA2[] = {0,0,0,1,0};      /**< Encender solo el tercer display, con transistor PNP, las centenas */
const int pinA3[] = {0,0,1,0,0};      /**< Encender solo el cuarto display, con transistor PNP, los miles */
const int pinA4[] = {0,0,1,1,1};      /**< Encender solo el quinto display, con transistor NPN, los diezmiles */

//Variables Flobales
unsigned long freq = 0;         /**< Variable que guarda la frecuencia leida para multiplexarla*/
unsigned long tasa = 1000000;   /**< Tasa de refresco que posteriormente se varia entres 2 opciones, 1000000us y 250000us */
unsigned long time;             /**< Variable que marca el tiempo cuando debe realizarse el refresco de la frecuencia*/

int unidades = 0;               /**< Variable para guardar el numero a mostrar en el display de las unidades*/
int decenas = 0;                /**< Variable para guardar el numero a mostrar en el display de las decenas*/
int centenas = 0;               /**< Variable para guardar el numero a mostrar en el display de las centenas*/
int miles = 0;                  /**< Variable para guardar el numero a mostrar en el display de las miles*/
int diezmil = 0;                /**< Variable para guardar el numero a mostrar en el display de las diezmil*/

const int inputPin = 2;                 /**< El pin donde se conecta la señal cuadrada para leer la frecuencia*/
volatile unsigned long pulseCount = 0;  /**< Se inicializa la variable que cuenta los pulsos en cada interrupción*/

void setup() {
  /**
  Se inicializa el sistema, se activan los pines que van a los 7 segmentos, tanto como a los transsitores y se establecen en modo salida.
  Con la función de arduino attachInterrupt, activa la interrupción al pin de entrada 2 y se llama a la funcion countPulse() en el flanco de subida
  @param Ninguno función no recibe parámetros
  @return Esta función no retorna nada
  */
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(inputPin, INPUT); 
  attachInterrupt(digitalPinToInterrupt(inputPin), countPulse, RISING); 
  Serial.begin(9600); /*! Se inicializa la comunicación serial para realizar debug*/
  time = micros();    /*! Con Time = micros() Se toma una medida de tiempo inicial, que posteriormente se cambia para contar la tasa*/

}

void loop() {
  /**
    Función que se ejecuta en loop infinito en el arduino, tomando la frecuencia cada que la tasa lo define, y la multiplexa en los 7 segmentos
    micros()-time)>tasa = Condición de la tasa de refresco para leer el dato de la frecuencia, micros es el valor actual de tiempo de ejecución, 
    mientras que time se actualiza cada que ingresa a refrescar la frecuencia
    Se desactiva las interrupciones, se reinicia la variable volatil pulseCount, y se llama la funcion espera() para contar los pulsos que me dan la frecuencia
    Luego se asigan los pulsos a la variable freq, se activan las interrupciones para continuar la ejecución de los relojs, y se toma una nuevva medida de tiempo

  */
  // if(Serial.available()>0){
  //    freq = (Serial.readStringUntil('\n').toInt());
  // }
  if((micros()-time)>tasa){     
      noInterrupts();           /*! Detengo interrupciones */
      pulseCount = 0;           /*! Reinicia el contador de pulsos */
      espera(1000);             /*! Ventana de un 1000 ms =  1 segundo donde cuento los pulsos de subida presentes, llamando la funcion espera() */
      freq = pulseCount;        /*! Se asigna a la variable freq la cantidad de pulsos presentes durante un segundo */
      interrupts();             /*! Se activan las interrupciones*/
      time = micros();          /*! Se toma una nueva medida de tiempo para continuar contando la tasa a la que se refresca la frecuencia*/
  }
  eval();                       /*! Llamo la función eval() porque dada la frecuencia debo actualizar la nueva tasa de refresco */
  multiplexar(freq);            /*! Multiplexo la frecuencia en los 7 segmentos, llamando la funcion multiplexar() */

}

void eval()
{
    /**
    Función que me actualiza el valor de la tasa a la cual leo la frecuencia nuevamente
    @param Ninguno
    @return No retorna pero modifica la variable global tasa
    */
    if(freq<= 4 ){            
        tasa = 1000000;       /*! if(freq<= 4 ) Actualizo la tasa a un segundo si la frecuencia es menor a 4Hz*/
    }else if(freq>4){         
        tasa = 250000;        /*! if(freq> 4 ) Actualizo la tasa a un 250ms si la frecuencia es mayor a 4Hz*/
    }
}

void espera(int valor)        
{
    /**
    Función que me permite esperar durante un segundo que se cuenten los pulsos sin dejar de multiplexar los displays
    @param int valor, el cual es un numero en milisegundos, el cual me indica el tiempo que se crea el loop para contar pulsos
    @return No retorna, pero cada vez que hay una interrupción se llama la funcion countPulse()
    */
    unsigned long int Contador = millis() + valor;  /**contador < Variable contador local, que me indica el tiempo actual + la cantidad de tiempo en ms que se hace el loop*/
    interrupts();             /*! Activo interrupciones*/
    do { multiplexar(freq);   /*! Continúo multiplexando mientras se cuentan pulsos y se cumple que Contador >= millis(), es decir un segundo*/
        } while (Contador >= millis());
    noInterrupts();         /*! Desactivo interrupciones*/
}

void countPulse() 
{
  /**
  Función que se ejecuta cada vez que hay una interrupción, es decir un flanco de subida en el pin 2 del arduino
  @param Ninguno
  @return No retorna nada, solo modifica la variable global volatil pulseCount
  */
  pulseCount++;     /*! Incremento los pulsos cada vez que hay un flanco de subida, es decir cada interrupción*/
}

void segmentos(int valor[])
{
  /**
  Enciendo los leds de acuerdo al arreglo de enteros, habilitando las salidas digitales para el display, al ser ánodo común se habilitan con 0
  @param Arreglo de enteros, donde cada posicion equivale a un segmento en el orden establecido
  @return no retorna nada, solo cambia las salidas digitales de la 5 a la 11
  */
  digitalWrite(5, valor[0]);    /*! valor[0] = segmento a */
  digitalWrite(6, valor[1]);    /*! valor[1] = segmento b */
  digitalWrite(7, valor[2]);    /*! valor[2] = segmento c */
  digitalWrite(8, valor[3]);    /*! valor[3] = segmento d */
  digitalWrite(9, valor[4]);    /*! valor[4] = segmento e */
  digitalWrite(10, valor[5]);   /*! valor[5] = segmento f */
  digitalWrite(11, valor[6]);   /*! valor[6] = segmento g */
}
void off()                      
{
  /**
  Función que apaga todos los segmentos del display
  @param Ninguno
  @return No retorna nada, pone en 1 todas las salidas digitales de la 5 a la 11
  */
  digitalWrite(5, 1);
  digitalWrite(6, 1);
  digitalWrite(7, 1);
  digitalWrite(8, 1);
  digitalWrite(9, 1);
  digitalWrite(10, 1);
  digitalWrite(11, 1);
}

void asingpin(int pin[])    
{
  /**
  Función que activa un display a la vez y apaga los demás
  Cambia la salida Analoga que actúa como digital en este caso a 1 o 0 según el valor en la posición del arreglo
  Estas salidas controlan que display se enciede o apaga saturando el transistor correspondiente y dejando circular VCC
  @param Arreglo de enteros
  @return No retorna 
  */
  digitalWrite(A0, pin[0]); /*!pin[0] = unidades*/
  digitalWrite(A1, pin[1]); /*!pin[1] = decenas*/
  digitalWrite(A2, pin[2]); /*!pin[2] = centenas*/
  digitalWrite(A3, pin[3]); /*!pin[3] = miles*/
  digitalWrite(A4, pin[4]); /*!pin[4] = diezmiles*/
  delay(1);                 /*!Con un delay de 1 ms Dejo el display encendido un corto tiempo*/
  off();                    /*!Luego apago todos los leds, para evitar que aparezca otro numero en los displays con la funcion off()*/
}

void select(int numero)         
{
    /**
    Función que selecciona que numero escribir en los leds, asignando a cada caso el numero correspondiente en la variable con el arreglo
    @param int numero, el cual es un numero entero entre 0 y 11, para saber que numero se muestra en el display, los numeros 10 y 11
    son comodines para dibujar la linea inferior para frecuencias menores a 1Hz y linea superior para frecuencias mayores 50KHz
    @return No retorna, solo llama la funcion segmentos() y les pasa el nombre de la variable global con el número a dibujar
    */
    switch (numero){
    case 0:
      segmentos(cero);
      break;
    case 1:
      segmentos(one);
      break;
    case 2:
      segmentos(two);
      break;
    case 3:
      segmentos(three);
      break;
    case 4:
      segmentos(four);
      break;
    case 5:
      segmentos(five);
      break;
    case 6:
      segmentos(six);
      break;
    case 7:
      segmentos(seven);
      break;
    case 8:
      segmentos(eight);
      break;
    case 9:
      segmentos(nine);
      break;
    case 10:
      segmentos(menor);
      break;
    case 11:
      segmentos(mayor);
      break;
    }
}

void multiplexar(unsigned long numero)     
{
    /**
    Función que realiza la multiplexación de los numeros en los displays
    @param unsigned long numero, es la frecuencia a multiplexar
    @return No retorna nada, ni modifica variables
    Después de evaluar si dibujar el numero, con la función select() le paso la variable global correspondiente a dibujar en los leds,
    y activo el display del 1 al 5 correspondiente, ya se unidades, decenas, centenas, miles, diezmiles con la función asingpin()
    */

    /*! Se descompone primero la frecuencia en sus unidades, decenas, centenas, miles y diezmiles, llamo a la funcion splitnumber() para ello*/
    splitnumber(numero);                    
    /*! Evalúo si dibujar las unidades, esto porque debo dibujar un cero en caso de que sea una frecuencia mayor a 10*/
    if(unidades != 0 | decenas >0 | centenas >0 | miles>0 | diezmil >0 && diezmil <6){
      select(unidades);
      asingpin(pinA0);
    }
    /*! Evalúo si dibujar las decenas, esto porque debo dibujar un cero en caso de que sea una frecuencia mayor a 100*/
    if(decenas !=0 | centenas >0 | miles>0 | diezmil >0 && diezmil <6){
      select(decenas);
      asingpin(pinA1);
    }
    /*! Evalúo si dibujar las centenas, esto porque debo dibujar un cero en caso de que sea una frecuencia mayor a 1000*/
    if(centenas !=0 | miles>0 | diezmil >0 && diezmil <6){
      select(centenas);
      asingpin(pinA2);
    }
    /*!Evalúo si dibujar los miles, esto porque debo dibujar un cero en caso de que sea una frecuencia mayor a 10000*/
    if(miles !=0 | diezmil >0 && diezmil <6){    

      select(miles);
      asingpin(pinA3);
    }
    /*!Evalúo si dibujar los diezmiles, esto porque debo dibujarlas solo si son frecuencias menores a 50000*/
    if(diezmil !=0 && diezmil <6){      
      select(diezmil);
      asingpin(pinA4);
    }
    /*!En caso de ser una frecuencia menor a 1Hz selecciono el numero 10 que dibuja solo el segmento d de los displays y los muestro en todos*/
    if(unidades == 0 && decenas == 0 && centenas == 0 && miles == 0 && diezmil == 0){ 
      select(10);
      asingpin(pinA0);
      select(10);
      asingpin(pinA1);
      select(10);
      asingpin(pinA2);
      select(10);
      asingpin(pinA3);
      select(10);
      asingpin(pinA4);
    }
    /*!En caso de ser una frecuencia mayor a 50KHz selecciono el numero 11 que dibuja solo el segmento a de los displays y los muestro en todos*/
    if(unidades == 9 && decenas == 9 && centenas == 9 && miles == 9 && diezmil == 9){
      select(11);
      asingpin(pinA0);
      select(11);
      asingpin(pinA1);
      select(11);
      asingpin(pinA2);
      select(11);
      asingpin(pinA3);
      select(11);
      asingpin(pinA4);
    }
     
}

void splitnumber(unsigned long numero)  
{
  /**
  Función que descompone un numero, en este caso la frecuencia, en sus unidades, decenas, centenas, miles, diezmiles
  @param numero, el cual es la frecuencia a descomponer
  @return No retorna, solo modifica las variables globales unidades, decenas, centanas, miles, diezmil
  */
  if(numero < 1){                       /*!if (numero < 1) Si el número es menor a 1, es decir una frecuencia de cero, asigno inmediatamente 0 a todas las variables*/
     unidades = decenas =centenas = miles = diezmil = 0;
  }else if(numero >=1 && numero <= 50000){ /*! Si la frecuencia está entre 1Hz y 50KHz descompongo el número*/
    unidades = numero % 10;              /*! unidades = numero % 10 ; Obtenemos las unidades*/
    decenas = (numero / 10) % 10;        /*! decenas = (numero / 10) % 10; Obtenemos las decenas*/
    centenas = (numero / 100) % 10;      /*! centenas = (numero / 100) % 10;  Obtenemos las centenas*/
    miles = (numero / 1000) % 10;        /*! miles = (numero / 1000) % 10; Obtenemos los miles*/
    diezmil = (numero / 10000) % 10;     /*! diezmil = (numero / 10000) % 10; Obtenemos los diezmiles */
  }else if(numero >51000){
    unidades = decenas =centenas = miles = diezmil = 9; /*! Si el número es mayor a 50000, es decir una frecuencia mayor a 50KHz, asigno inmediatamente 9 a todas las variables*/
  }
  
}