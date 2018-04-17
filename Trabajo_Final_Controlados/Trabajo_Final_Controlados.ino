
#include <avr/interrupt.h> //Esto lo pongo porque decía el manual de avr que
#include <avr/io.h>        //supuestamente lo necesito para las interrupciones

#include "Controlados.h" //Acá están las funciones propias para no tener todo mezclado.
//Controlados es la clase. Ahora necesito crear objetos de esta clase:
Controlados controlados1;

// #   #   #   # Definiciones 
#define interruptON bitSet(SREG,7) //habilita las interrupciones globales
#define interruptOFF bitClear(SREG,7)//desactiva las interrupciones globales
// #   #   #   # Constantes 
const int cantMarcasEncoder = 8; //Es la cantidad de huecos que tiene el encoder de cada motor.
const int FsEncoders = 2000;
const int preescaler = 32;
const int cota = 400;//cota=32 hace que de 0 a aprox 100rpm asuma que la velocidad es cero.
const unsigned long _OCR2A = 250;



// ############# Probandos cosas locas
#define NOP __asm__ __volatile__ ("nop\n\t")
#define SalidaTest 3
bool estado=0,estado2=0,estado3=0;
 //unsigned long suma=0;//float suma=0;
 
//################

// #   #   #   # Variables
unsigned char trama_activa=0;//Lo pongo en unsigned char para que ocupe 1 byte (int ocupa 2)                           
int cantOVerflow=0;//Variable que almacena la cantidad de veces que se desbordó el timer2 hasta que vuelvo a tener interrupción por pin de entrada. Esto permite realizar la medición de 
                  //tiempos entre aujeros del encoder.                                                                              
/*
int TCNT2anterior=0;//Valor anterior del contador (para corregir la medición)
int TCNT2actual=0;//Almaceno el valor del timer para que no me jodan posibles actualizaciones.                
int cantOVerflow_actual=0;     //Valor anterior del contador (para corregir la medición), correspondiente al TCNT2anterior.          
*/
unsigned long TCNT2anterior=0;//Valor anterior del contador (para corregir la medición)
unsigned long TCNT2actual=0;//Almaceno el valor del timer para que no me jodan posibles actualizaciones.                
unsigned long cantOVerflow_actual=0;     //Valor anterior del contador (para corregir la medición), correspondiente al TCNT2anterior. 
unsigned long aux[6];         
unsigned long  bufferVel;//buffer donde almaceno las últimas velocidades calculadas.
float velAngular=0;//última velocidad angular calculada. Lo separo del vector bufferVel para que no haya problemas por interrumpir en medio de una actualización de éste.
float velDeseada=0;//Empieza detenido el motor.

// #   #   #   # Variable Basura

int Bandera=0; // bandera para administrar procesos fuera de interrupciones  
// #   #   #   # Declaracion de Funciones

void medirVelocidad(unsigned char);
void EnviarTX(int cantidad,char identificador, unsigned long *datos);
void EnviarTX_online(unsigned long var);

void setup() {
  interruptOFF; // se desactivan las interrupciones para configurar.
  Serial.begin(115200);
  //controlados1.configPinesMotores();
 // controlados1.modoStop();
  //controlados1.configTimerMotores();
  controlados1.configTimer2Contador(FsEncoders,preescaler,1);//Configuro el timer2 como contador con interrupción. La frecuencia va desde 500000 hasta 1997.   
  //controlados1.actualizarDutyCycleMotores(70,30);
 // controlados1.modoAdelante();

  interruptON;//Activo las interrupciones
  pinMode(A0, INPUT);
  // $Prueba
  pinMode(SalidaTest, OUTPUT);
   //configTimer2Contador(); // la agrego a mano.
}

void loop() {
  NOP;

if (bitRead(Bandera,0)){ // timer 2 overflow
  } 
  if (bitRead(Bandera,1)){ // Entra cuando no registra cambio en la entrada
  // Serial.println  (Bandera,BIN);
  bitWrite(Bandera,1,0);
  medirVelocidad(0);   

  } 
  if (bitRead(Bandera,2)){ // se registra cambio en la entrada
  //  Serial.println  (Bandera,BIN);
  bitWrite(Bandera,2,0);
  medirVelocidad(1);  
  } 
   
}

void serialEvent() { // esta funcion se activa sola, es por interrupciones (ponele)
  int dato;
  if (Serial.available() > 0) {
    dato= Serial.read();
    switch (dato){
    case 0xFF:
    trama_activa=1;
    break;
    case 0xFE:
    trama_activa=2;
    break;
    case 253:
    trama_activa=3;
    break;
    case 252:
    trama_activa=4;
    break;
    default:
    trama_activa=0;
    break;

    }
  }
}

ISR (TIMER2_COMPA_vect){//Interrupción por Timer2 para definir frec de muestreo del sist cte; Resetea con el valor de comparacion del A
  cantOVerflow++;
  if(cantOVerflow>cota){
   bitWrite(Bandera,1,1); //medirVelocidad(0);//Llamo a la rutina de medición de vel indicándole que pasó demasiado tiempo y que tiene que asumir que la velocidad es 0.                                        
  }
  bitWrite(Bandera,0,1);
  //estado=!estado;
  //if (estado){
    estado2=!estado2;
  digitalWrite(SalidaTest,estado2);
  //}
  
}
ISR(PCINT1_vect){
 /*
  * Hay que verificar donde fue el cambio de estado, porque al tener 2 ruedas, no se sabe de donde provino (habria que hacer una comparacion manual)
  */
  TCNT2anterior=TCNT2actual;//Ahora el valor actual pasa a ser el anterior de la próxima interrupción.                           
  TCNT2actual=TCNT2;//Almaceno enseguida el valor del timer para que no cambie mientras hago las cuentas.                   
  cantOVerflow_actual=cantOVerflow; 
  cantOVerflow=0;
  bitWrite(Bandera,2,1);   //medirVelocidad(1); 
 
}

void medirVelocidad(unsigned char interrupcion)
{
  long t;
  long tmh;
 // cantOVerflow=0;//Reseteo el valor de cantidad de interrupciones ocurridas por timer 2
  if(interrupcion){
     t=cantOVerflow_actual*_OCR2A;
     tmh=TCNT2actual-TCNT2anterior+t;
    bufferVel=long(preescaler)*(tmh);
    //bufferVel=long(preescaler)*(cantOVerflow_actual*long(OCR2A)+TCNT2actual-TCNT2anterior);
  }
  else{
    bufferVel=0;
  }   
  aux[0]=TCNT2anterior;
  aux[1]=TCNT2actual;
  aux[2]=cantOVerflow_actual;   
  aux[3]=bufferVel; 
  aux[4]= tmh;
  aux[5]= t;                     
EnviarTX(6,'a',aux);
EnviarTX_online(bufferVel);
}

// Ver como mejorar sustancialmente esta funcion. Esta media fea.
void EnviarTX(int cantidad,const char identificador, unsigned long *datos){
  
 // [inicio][cantidad de datos][identificador][Datos][fin]
 if (trama_activa==3){ 
  Serial.println(0xFF,DEC);
  Serial.println(cantidad,DEC);
  Serial.println(identificador);
  float a;
for (int i=0;i<cantidad;i++){
  a=*(datos+i);
   Serial.println(a,DEC);
  }
  
  Serial.println(0xFE,DEC);
 }
   
  }

void EnviarTX_online(unsigned long var){
  if (trama_activa==4){
  Serial.println(var,DEC);
  }
  }

