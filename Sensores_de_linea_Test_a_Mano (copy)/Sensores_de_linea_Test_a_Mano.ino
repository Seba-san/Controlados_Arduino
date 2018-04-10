



#include <Controlados.h>; //Acá están las funciones propias para no tener
                          //todo mezclado.
//Controlados es la clase. Ahora necesito crear objetos de esta
//clase:
Controlados controlados1;

#define interruptON bitSet(SREG,7) //habilita las interrupciones globales
#define interruptOFF bitClear(SREG,7)//desactiva las interrupciones globales

int aux1,aux2,error;

#define N 500 //Cantidad de muestras a tomar
int muestras[N];
#define Taprox 1000 //Periodo de muestreo aproximado

unsigned char muestra,muestra2;

struct error {
  int aa,bb;
  float cc;
};

void setup() {
  interruptOFF; // se desactivan las interrupciones para configurar.
  Serial.begin(9600); // opens serial port, data rate 9600 bps.

  //Config del motor:
  controlados1.configPinesMotores();
  controlados1.modoStop();
  controlados1.configTimerMotores();

  //Config Sensor de línea:
  controlados1.configPinesSensorLinea();
  interruptON;//Hay que activarlas para que funcione la funcion "delay()".
  controlados1.actualizarDutyCycleMotores(70,70); 
  //controlados1.modoAdelante();
 
}

void loop() {

   struct error error2;
  aux1=PIND&B00011100;//Me quedo con D4, D3, D2
  //aux1=PIND&28;//28=&B00011100
  aux2=PINC&B01111100;//Me quedo con A6 al A2
  muestra=(aux1<<3)|(aux2>>2);

  aux1=analogRead(A6);
  aux2=aux1&512;
  muestra=muestra|(aux2>>5);
  muestra2=~muestra;
  //muestra=muestra|B10000000;
  error2 = calc_error(muestra2); // hay que medir el tiempo que tarda esta funcion.
  Serial.print(" ");
  Serial.print(muestra,BIN);
  Serial.print(" ");
  Serial.print(error2.aa,DEC);
  Serial.print(" ");
  Serial.print(error2.bb,DEC);
  Serial.print(" ");
  Serial.println(error2.cc,DEC);
//  muestra=controlados1.leerSensorDeLinea();
//  Serial.print("   ");
//  Serial.println(muestra|B10000000,BIN);

  delay(1000);
  //delayMicroseconds(Taprox);//Periodo de muestreo aproximado
}



struct error calc_error(unsigned char entrada) {
  int cuenta,a,b,c,salida;
  struct error salidaa; // armar una estructura permite obtener una funcion con muchas salidas. Otra opcion es usar punteros, ver:http://forum.arduino.cc/index.php?topic=92011.0
  salidaa.aa=0;
  salidaa.bb=0;
  salidaa.cc=0;
  a=0;b=0;
 for (int i=0; i <= 7; i++)
 {
      c=bitRead(entrada,i); // leo un bit espesifico de la variable.
      a=a+c;
      b=b+c*(i-3);   
 }
   if (a==0) { // no se contempla el caso de error, ejemplo 1101011.
    a=1;
   }
   salidaa.aa=a;
   salidaa.bb=b;
    salidaa.cc=(float)b/a; // tiene que ser float, porque los numeros tienen coma :-).
    return salidaa;  
     
  }

  


