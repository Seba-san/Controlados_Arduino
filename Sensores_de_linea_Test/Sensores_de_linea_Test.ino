//interrupciones, fue extraido de AtmelSutdio 7

#include <Controlados.h>; //Acá están las funciones propias para no tener
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

  delay(5000);
  controlados1.actualizarDutyCycleMotores(20,20); 
  controlados1.modoAdelante();
  delay(1000);
  for (int n=0; n < N; n++)
  {
    aux1=PIND&B00011100;//Me quedo con D4, D3, D2
    aux2=PINC&B01111100;//Me quedo con A6 a A2
    //a << n corre los bits de a n lugares a la izq, mientras
    //que a >> n los corre a la derecha.
    muestra=(aux1<<3)|(aux2>>2);
    //Lectura del bit analógico:
    aux1=analogRead(A6);
    aux2=aux1&512;
    muestra=muestra|(aux2>>5);
    //Fin lectura bit analógico
    muestras[n]=muestra;
    delayMicroseconds(Taprox);//Periodo de muestreo aproximado
  }
  controlados1.modoStop();
}

void loop() {
  //Nada
}

void serialEvent() {
  int dato;
  if (Serial.available() > 0) {
    dato= Serial.read();
    if (dato==0x01){//Le mando todas las muestras guardadas      
      for (int n=0; n < N; n++){
        Serial.println(muestras[n]|B10000000,BIN);
        delayMicroseconds(Taprox);//Periodo de muestreo aproximado
      }
    }
  }
}
