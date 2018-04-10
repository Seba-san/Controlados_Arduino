#include <Controlados.h> //Acá están las funciones propias para no tener
                          //todo mezclado.
//Controlados es la clase. Ahora necesito crear objetos de esta
//clase:
Controlados controlados1;

#define interruptON bitSet(SREG,7) //habilita las interrupciones globales
#define interruptOFF bitClear(SREG,7)//desactiva las interrupciones globales

int aux1;
int aux2;
int muestra;

#define N 100 //Cantidad de muestras a tomar
int muestras[N];
#define Taprox 10000 //Periodo de muestreo aproximado

void setup() {
  interruptOFF; // se desactivan las interrupciones para configurar.
  Serial.begin(9600); // opens serial port, data rate 9600 bps.

  //Config del motor:
  controlados1.configPinesMotores();
  controlados1.modoStop();
  controlados1.configTimerMotores();

  //Config Sensor de línea:
  controlados1.configPinesSensorLinea();
  interruptON;//Activo las interrupciones
}

void loop() {
  //Nada
}

void serialEvent() {
  int dato;
  if (Serial.available() > 0) {
    dato= Serial.read();
    digitalWrite(LED_BUILTIN,HIGH);
    Serial.print(dato);
  }
}
