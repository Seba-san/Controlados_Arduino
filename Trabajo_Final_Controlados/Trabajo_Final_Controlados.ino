//Tania estuvo aquí y seba tambien
//probando cosas
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
const int FsEncoders = 2000;//8000 2000
const int preescaler = 32;//8 32 64 
const int cota = 400;//cota=32 hace que de 0 a aprox 100rpm asuma que la velocidad es cero.
const unsigned long _OCR2A = 250;
// F_CPU es el valor con el que esta trabajando el clock del micro.


// ############# Probandos cosas locas 
#define NOP __asm__ __volatile__ ("nop\n\t")
#define SalidaTest 3
#define SalidaTest2 4
#define SalidaTest3 5
bool estado=0,estado2=0,estado3=0, estado4=0;
 //unsigned long suma=0;//float suma=0;
 
// #   #   #   # Variable Basura
int Bandera=0; // bandera para administrar procesos fuera de interrupciones 
float b, freq; 
unsigned long cuenta;    
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
unsigned long aux[6];  // este es un buffer para enviar datos en formato trama, corresponde a la funcion "EnviarTX"       
unsigned long  bufferVel[2*cantMarcasEncoder];//buffer donde almaceno las últimas velocidades calculadas.
float velAngular=0;//última velocidad angular calculada. Lo separo del vector bufferVel para que no haya problemas por interrumpir en medio de una actualización de éste.
float velDeseada=0;//Empieza detenido el motor.


// #   #   #   # Declaracion de Funciones

void medirVelocidad(unsigned char);
void EnviarTX(int cantidad,char identificador, unsigned long *datos);
void EnviarTX_online(unsigned long var);
void timer_interrupt(void);

void setup() {
  interruptOFF; // se desactivan las interrupciones para configurar.

Serial.begin(115200); // Si se comunica a mucha velocidad se producen errores (que no se detectan hasta que haces las cuentas)

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
  pinMode(SalidaTest2, OUTPUT);
  pinMode(SalidaTest3, OUTPUT);

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

timer_interrupt(); 

}
void timer_interrupt(){ // se utiliza esta funcion ya que permite llamarla desde otras funciones. Si lo haces desde la interrupcion, no podes.
  //estado2=!estado2;
  //digitalWrite(SalidaTest,estado2);
  cantOVerflow++;
  if(cantOVerflow>cota){
   bitWrite(Bandera,1,1); //medirVelocidad(0);//Llamo a la rutina de medición de vel indicándole que pasó demasiado tiempo y que tiene que asumir que la velocidad es 0.                                        
  }
  bitWrite(Bandera,0,1);
  //estado=!estado;
  //if (estado){
   // estado2=!estado2;
  //digitalWrite(SalidaTest,estado2);
  //}
  //digitalWrite(SalidaTest,0);
}
ISR(PCINT1_vect){
 /*
  * Hay que verificar donde fue el cambio de estado, porque al tener 2 ruedas, no se sabe de donde provino (habria que hacer una comparacion manual)
  */
 // estado3=!estado3;
 // if (estado3){
 //digitalWrite(SalidaTest2,estado3);
  TCNT2anterior=TCNT2actual;//Ahora el valor actual pasa a ser el anterior de la próxima interrupción.   
  if (bitRead(TIFR2,1)){ // me fijo si hay overflow
  timer_interrupt();
  bitSet(TIFR2,1); // borro bandera para que no entre de nuevo
  }                        
  TCNT2actual=TCNT2;//Almaceno enseguida el valor del timer para que no cambie mientras hago las cuentas.                   
  cantOVerflow_actual=cantOVerflow; 
  cantOVerflow=0;
  bitWrite(Bandera,2,1);   //medirVelocidad(1); 
  //estado=!estado;
   //digitalWrite(SalidaTest2,0);
  //}
  
}

void medirVelocidad(unsigned char interrupcion)
{


  
unsigned long suma=0;
 long t,tmh; // Lo hago por partes porque todo junto generaba errores en algunas ocaciones
  //Corro los valores de w en el buffer un lugar y voy sumando los valores para después calcular el promedio de velocidades:  
  for(int k=0;k<(2*cantMarcasEncoder-1);k++)
  {
    bufferVel[k]=bufferVel[k+1];//Desplazamiento a la derecha de los datos del buffer
    suma=suma+bufferVel[k];
  }
  
  //Al terminar el bucle bufferVel tiene los últimos dos valores iguales (los dos de más a la izquierda). Esto cambia a continuación con la actualización del valor más a la derecha:
  if(interrupcion){
    // Se hace de forma separada porque se detectaron problemas de calculo asociado con los tipos de variables. Esto se resolvio separando las cuentas, queda a futuro resolverlo en una sola linea.
     t=cantOVerflow_actual*OCR2A;
     tmh=TCNT2actual-TCNT2anterior+t; // Ver problemas de variables
    bufferVel[2*cantMarcasEncoder-1]=long(preescaler)*(tmh);
    
    //bufferVel[2*cantMarcasEncoder-1]=(long(preescaler)*(TCNT2actual+cantOVerflow_actual*long(OCR2A)-TCNT2anterior));
     //bufferVel[2*cantMarcasEncoder-1]=float(fclkio)/(float(preescaler)*(TCNT2actual+cantOVerflow_actual*float(OCR2A)-TCNT2anterior));
   // bufferVel[2*cantMarcasEncoder-1]=preescaler*(TCNT2actual+cantOVerflow*OCR2A-TCNT2anterior)*fclkio2;// Es para contar ciclos
    //bufferVel[2*cantMarcasEncoder-1]=fclkio/(2*cantMarcasEncoder*preescaler*(TCNT2actual+cantOVerflow*OCR2A-TCNT2anterior));
                              //Unidad de medición: ciclos/seg.
   // bufferVel[2*cantMarcasEncoder-1]=60.0*float(fclkio)/(float(2*cantMarcasEncoder*preescaler)*(float(cantOVerflow_actual*OCR2A)-TCNT2anterior+TCNT2actual));
                              //Unidad de medición: rpm.
  }
  else{
     bufferVel[2*cantMarcasEncoder-1]=0;
  }   

// ################## Parte del codigo funcional con la version: commit b162d5d7e01e02d2daadd90df3ba0800cd0464f5
  freq=float(F_CPU)/(bufferVel[15]+bufferVel[14]);
  b=100-freq;
  if (abs(b)>1){
    //estado4=!estado4;
     // digitalWrite(SalidaTest3,estado4);
  aux[0]=TCNT2anterior;
  aux[1]=TCNT2actual;
  aux[2]=cantOVerflow_actual;   
  aux[0]= bufferVel[15]+bufferVel[14]; 
    aux[1]= cuenta; 
    cuenta=0;                 
EnviarTX(2,'a',aux);
  }
  else{cuenta++;}
EnviarTX_online( bufferVel[2*cantMarcasEncoder-1]);

// ################## Parte del codigo funcional con la version: commit b162d5d7e01e02d2daadd90df3ba0800cd0464f5
}

// Ver como mejorar sustancialmente esta funcion. Esta media fea.
void EnviarTX(int cantidad,const char identificador, unsigned long *datos){ // ojo, datos es un puntero
  
 // [inicio][cantidad de datos][identificador][Datos][fin] Agregar un CRC y ACK???

 if (trama_activa==3){ 

  Serial.println(0xFF,DEC); // inicio
  Serial.println(cantidad,DEC);// cantidad de datos
  Serial.println(identificador);// identificador, se puede sacar, pero para fines debuggeros puede ayudar
  float a;
for (int i=0;i<cantidad;i++){
  a=*(datos+i); // como datos es un puntero, necesito guardar en "a" el apuntador.
   Serial.println(a,DEC);
  }
  
  Serial.println(0xFE,DEC); // fin
 }
   
  }

void EnviarTX_online(unsigned long var){ // funcion de envio de datos de corta duracion. No se envia en formato trama, solo verifica una bandera.
  if (trama_activa==4){
  Serial.println(var,DEC);
  }
  }

