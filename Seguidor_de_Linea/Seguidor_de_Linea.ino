#include <Controlados.h>; //Acá están las funciones propias
                          //para no tener todo mezclado.
//Controlados es la clase. Ahora necesito crear objetos de esta
//clase:
Controlados controlados1;

#define interruptON bitSet(SREG,7)//habilita las interrupciones 
                                  //globales
#define interruptOFF bitClear(SREG,7)//desactiva las 
                                     //interrupciones globales



void setup() {
  interruptOFF; //se desactivan las interrupciones para 
                //configurar.
  Serial.begin(9600);//opens serial port, data rate 9600 bps.

  //Config del motor:
  controlados1.configPinesMotores();
  controlados1.modoStop();
  controlados1.configTimerMotores();

  //Config Sensor de línea:
  controlados1.configPinesSensorLinea();
  interruptON;//Activo las interrupciones


}

void loop() {
  // put your main code here, to run repeatedly:

}
