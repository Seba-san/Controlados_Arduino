//#include "interrupt.h" // Necesario para manipular facilmente  las
//interrupciones, fue extraido de AtmelSutdio 7

//#include <Controlados.h>; //Acá están las funciones propias para no tener
//todo mezclado.
//Controlados es la clase. Ahora necesito crear objetos de esta
//clase:
//Controlados controlados1;

#define interruptON bitSet(SREG,7) //habilita las interrupciones globales
#define interruptOFF bitClear(SREG,7)//desactiva las interrupciones globales
#define salida 13 //Salida analógica donde pongo el PWM, es decir, para
                  //controlar la sensibilidad del sensor
#define entrada A4 //Entrada analógica. Para poner el detector

//Modificar todo esto junto:
#define salidaDigital 12 
#define puertoSalida PORTB
#define bitSalida 4

//AJUSTAR!!!
#define umbralLow 100 //Va de 0 a 1023
#define umbralHigh 200

#define umbralMax 818//Interpreto como uno si V>=umbralMax=4V aprox
#define umbralMin 205//Interpreto como cero si V<=umbralMin=1V aprox

long valor = 43;
int dato = 0;
int nivel;
int estado = 0; //Arranco suponiendo estado incial bajo

void setup() {
  interruptOFF; // se desactivan las interrupciones para configurar.
  Serial.begin(9600); // opens serial port, data rate 9600 bps.

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(salida, OUTPUT);
  pinMode(salidaDigital, OUTPUT);
  pinMode(entrada, INPUT);
  //controlados1.configTimerMotores();
  interruptON;//Activo las interrupciones
  //Serial.println(0);
  analogWrite(salida, valor);
}

void loop() {
  //digitalWrite(LED_BUILTIN,bitRead(PORTB, 1));
  dato = analogRead(entrada);
  if (estado == 0 && dato > umbralHigh) {
    estado = 1;
    bitSet(puertoSalida,bitSalida);
  }
  else if (estado == 1 && dato < umbralLow) {
    estado = 0;
    bitClear(puertoSalida,bitSalida);
  }
}

void serialEvent() {
  //int dato;

  if (Serial.available() > 0) {
    //dato= Serial.read();
    valor = Serial.read();
    analogWrite(salida, valor); //analogWrite(salida,valor), donde valor es de 0 a 255 (de siempre apagado a siempre prendido)
    //controlados1.actualizarDutyCycleMotores(dato,dato);//Actualizo los dos PWM
    Serial.println(valor);//Eco para control
  }
}

//void ajustarBlanco(){
//  int a=0;
//  int b=255;
//  int n;
//  int Vin;
//  bool go=true;
//  while(go=true)
//  {
//    n=(b-a)/2;
//    analogWrite(salida,n);
//    Vin=analogRead(entrada);
//  }
//}

