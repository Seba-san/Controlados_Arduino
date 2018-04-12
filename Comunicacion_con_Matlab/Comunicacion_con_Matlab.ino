/*
Modificación de la versión inicial de Trabajos Final Controlados (versión del 12/04/18)
para probar la comunicación con Matlab
 */


#include <avr/interrupt.h> //Esto lo pongo porque decía el manual de avr que
#include <avr/io.h>        //supuestamente lo necesito para las interrupciones

#include "Controlados.h" //Acá están las funciones propias para no tener
                          //todo mezclado.
//Controlados es la clase. Ahora necesito crear objetos de esta
//clase:
Controlados controlados1;

#define interruptON bitSet(SREG,7) //habilita las interrupciones globales
#define interruptOFF bitClear(SREG,7)//desactiva las interrupciones globales
int borrar;


void setup() {
  interruptOFF; // se desactivan las interrupciones para configurar.
  Serial.begin(2000000);
  controlados1.configTimer2Contador((long) 2000);//Configuro el timer2 como contador con interrupción. La frecuencia va desde 500000 hasta 1997.   

  //Interrupciones por estado en pin para lectura de los encoders:
  bitWrite(PCICR,PCIE1,1); // Pin Change Interrupt Control Register ; Bit 1 – PCIE1: Pin Change Interrupt Enable 1; PCINT[14:8]
  bitWrite(PCMSK1,PCINT8,1); // PCINT8 correspondo a A0.
  bitWrite(PCIFR,PCIF1,1);// Limpio la bandera
  //Ver si esto lo ponemos en la librería$
  interruptON;//Activo las interrupciones
}

void loop() {
}

ISR (TIMER2_COMPA_vect){//Interrupción por Timer2 para definir frec de muestreo del sist cte; Resetea con el valor de comparacion del A
    borrar++;
    if(borrar>100){borrar=0;}
    Serial.println(borrar);
}


