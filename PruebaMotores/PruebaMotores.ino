//#include "interrupt.h" // Necesario para manipular facilmente  las
//interrupciones, fue extraido de AtmelSutdio 7

#include "Controlados.h"; //Acá están las funciones propias para no tener
                          //todo mezclado.
//Controlados es la clase. Ahora necesito crear objetos de esta
//clase:
Controlados controlados1;

#define interruptON bitSet(SREG,7) //habilita las interrupciones globales
#define interruptOFF bitClear(SREG,7)//desactiva las interrupciones globales
#define pulsador 2

int dato;
int aux=1;
int fin=0;

void setup() {
  interruptOFF; // se desactivan las interrupciones para configurar.
  Serial.begin(9600); // opens serial port, data rate 9600 bps.
  controlados1.configPinesMotores();
  controlados1.modoStop();
  controlados1.configTimerMotores();
  controlados1.actualizarDutyCycleMotores(70,30); 
  pinMode(pulsador,INPUT);
  pinMode(LED_BUILTIN,OUTPUT);
  interruptON;//Activo las interrupciones
}

void loop() {
//  aux=digitalRead(pulsador);
//  if (aux==0)
//  {
//    digitalWrite(LED_BUILTIN,LOW);
//    controlados1.modoAdelante();
//  }
//  else
//  {
//    digitalWrite(LED_BUILTIN,HIGH);
//    controlados1.modoStop();
//  }
  
  while(fin==0)
  {
    delay(3000);
    controlados1.modoAdelante();
    delay(1500);
    controlados1.modoStop();
    delay(1000);
    controlados1.modoAtras();
    delay(1500);
    controlados1.modoStop();
    fin=1;
  }
}

void serialEvent() {
  int dato;
  if (Serial.available() > 0) {
    dato= Serial.read();
    controlados1.actualizarDutyCycleMotores(dato,dato);//Actualizo los dos PWM
    Serial.println(dato);//Eco para control
  }
}
