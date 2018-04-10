#include <avr/interrupt.h> //Esto lo pongo porque decía el manual de avr que
#include <avr/io.h>        //supuestamente lo necesito para las interrupciones

#include "Controlados.h"; //Acá están las funciones propias para no tener
                          //todo mezclado.
//Controlados es la clase. Ahora necesito crear objetos de esta
//clase:
Controlados controlados1;

#define interruptON bitSet(SREG,7) //habilita las interrupciones globales
#define interruptOFF bitClear(SREG,7)//desactiva las interrupciones globales
#define cantMarcasEncoder 8 //Es la cantidad de huecos que tiene el encoder de
                            //cada motor.
#define preescaler 32
#define cota 382//cota=32 hace que de 0 a aprox 100rpm asuma que la velocidad es cero.
#define fclkio 16300000//DEFINIR UNIDAD $

float ticc,tocc;//Ver si lo necesito (y a las fc del final) o no$
unsigned char trama_activa=0;//Lo pongo en unsigned char para que ocupe 1 
                             //byte (int ocupa 2)
unsigned char nro_controlador;
int cantOVerflow;//Variable que almacena la cantidad de veces que se desbordó
                   //el timer2 hasta que vuelvo a tener interrupción por pin de
                   //entrada. Esto permite realizar la medición de tiempos entre
                   //aujeros del encoder.
unsigned char preescalerPorSoft=0;//Esta variable me permite aumentar por soft el
                                  //preescalador del timer 2.
int TCNT2anterior=0;//Valor anterior del contador (para corregir la medición)
int TCNT2actual=0;//Almaceno el valor del timer para que no me jodan posibles 
                  //actualizaciones.
float bufferVel[2*cantMarcasEncoder];//buffer donde almaceno las últimas 
                                     //velocidades calculadas.
float velAngular=0;//última velocidad angular calculada. Lo separo del vector 
                   //bufferVel para que no haya problemas por interrumpir en 
                   //medio de una actualización de éste.
float velDeseada=0;//Empieza detenido el motor.
float error[2]={0,0},u[2]={0,0};

void setup() {
  interruptOFF; // se desactivan las interrupciones para configurar.
  cantOVerflow=0;
  Serial.begin(2000000); // opens serial port, data rate 9600 bps.
  controlados1.configPinesMotores();
  controlados1.modoStop();
  controlados1.configTimerMotores();
  controlados1.configTimer2Contador();//Configuro el timer2 como contador con interrupción
  controlados1.actualizarDutyCycleMotores(70,30); 
  controlados1.modoAdelante();

  //Interrupciones por estado en pin para lectura de los encoders:
  bitWrite(PCICR,PCIE1,1); // Pin Change Interrupt Control Register ; Bit 1 – PCIE1: Pin Change Interrupt Enable 1; PCINT[14:8] 
  //Ver si esto lo ponemos en la librería$
  interruptON;//Activo las interrupciones
  pinMode(A0, INPUT);
}

void loop() {
  
}

void serialEvent() {
  int dato;
  if (Serial.available() > 0) {
    dato= Serial.read();
    if (trama_activa==1)//La cadena ya esta identificada como actualización de velocidad
    {
      velDeseada=dato;//Guardo el nuevo valor de vel deseada
      trama_activa=0;
      return;
    }
    else if (trama_activa==2)//La cadena ya esta identificada como selección del controlador
    {
        nro_controlador=dato;//Actualizo el controlador a usar (1=PID, 0=ninguno)
        //if (nro_controlador==0){SetPWM(0);}//FALTA VER QUÉ HACER PARA PARAR EL MOTOR$$
        trama_activa=0;
    }
    if (dato==0xFF)     //0xFF=Actualizar velocidad
    {
        trama_activa=1;
    }
    else if (dato==0xFE)    //0xFE=Seleccionar controlador
    {
        trama_activa=2;
    }
  }
}

ISR (TIMER2_COMPA_vect)//Interrupción por Timer2 para definir frec de muestreo del sist cte
{ 
  //COMPLETAR$
  cantOVerflow++;
  if(cantOVerflow>cota){
    medirVelocidad(0);//Llamo a la rutina de medición de vel indicándole
                      //que pasó demasiado tiempo y que tiene que asumir
                      //que la velocidad es 0.
  }
  preescalerPorSoft++;;//Esta variable me permite aumentar por soft el preescalador del timer
  if(preescalerPorSoft>=10){
    error[1]=velDeseada-velAngular;//ek=wdeseado-wmedido
    u[1]=ControladorMotor(error[0],error[1],u[0]);
    error[0]=error[1];//ek-1. El error que se utilizo en este ciclo va a ser el error anterior
                      //para el proximo ciclo
    if(velDeseada==0){u[1]=0;}//Esta parte permitiría apagar el motor cuando funciona el control
    //Poner de nuevo cuando terminen las pruebas$$
    //Serial.println(velAngular);//Le envío a la compu el valor de velocidad actual
  
    //PARTE DE PRUEBA, BORRAR!!!!$$
    //Hay que fijar la cant de cifras!!!!!$$
    Serial.print(cantOVerflow);
    Serial.print(' ');//Caracter para separar datos (space=SP=dec32=0x20)
    Serial.println(0);//Simulo que envío la señal de control (siempre 0)
  }
}

float ControladorMotor(float ek_1,float ek, float uk_1)
{
//Esta rutina implementa el controlador para la velocidad del motor.
    float uk;
    if(nro_controlador){
      uk=uk_1+0.008276*ek-0.007094*ek_1;
    }
    else{
      //COMPLETAR$$
    }
    //FALTA LA MODIIFCACION EFECTIVA DE LA VELOCIDAD DEL MOTOR.
    return uk;
}

ISR(PCINT1_vect){
  medirVelocidad(1);
}
void medirVelocidad(unsigned char interrupcion)
{
  //Arranco determinando el conteo real de ciclos del timer2 tranascurridos desde la última 
  //interrupción:
  TCNT2actual=TCNT2;//Almaceno enseguida el valor del timer para que no cambie mientras hago
                    //las cuentas.
  int suma=0;
  //Corro los valores de w en el buffer un lugar y voy sumando los valores para después 
  //calcular el promedio de velocidades:
  for(int k=0;k++;k<2*cantMarcasEncoder-1)
  {
    bufferVel[k]=bufferVel[k+1];//Desplazamiento a la derecha de los datos del buffer
    suma=suma+bufferVel[k];
  }
  //Al terminar el bucle bufferVel tiene los últimos dos valores iguales (los dos de más a
  //la izquierda). Esto cambia a continuación con la actualización del valor más a la derecha:
  if(interrupcion){
    bufferVel[2*cantMarcasEncoder-1]=60*fclkio/(2*cantMarcasEncoder*preescaler*(TCNT2actual+cantOVerflow*OCR2A-TCNT2anterior));
                              //Unidad de medición: rpm.
  }
  else{
    //Si no vengo de la interrupción es porque ya pasó demasiado tiempo. En ese caso asumo
    //que el motor está quieto. Esto lo hago llamando desde la interrupción por timer2
    //(cuanto cantOverflow supera la cota permitida).
    bufferVel[2*cantMarcasEncoder-1]=0;
  }
  cantOVerflow=0;//Reseteo el valor de cantidad de interrupciones ocurridas por timer 2
  TCNT2anterior=TCNT2actual;//Ahora el valor actual pasa a ser el anterior de la próxima
                            //interrupción.
  suma=suma+bufferVel[2*cantMarcasEncoder-1];
  velAngular=suma/(2*cantMarcasEncoder);//Actualizo el valor de velocidad medida como el 
                                        //promedio de las últimas mediciones (todas las del
                                        //buffer).
  //Obs: para el correcto funcionamiento de la rutina se requiere que no haya interrupción
  //por overflow en el timer 2 durante la ejecución de estas instrucciones
}

void tic(){
  ticc=micros();
}

void toc(){
  tocc=micros()-ticc;    
}
