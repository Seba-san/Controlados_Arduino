//#include "interrupt.h" // Necesario para manipular facilmente  las
//interrupciones, fue extraido de AtmelSutdio 7

#include <RHReliableDatagram.h>
#include <RH_ASK.h>

#include <Controlados.h>; //Acá están las funciones propias para no tener
                          //todo mezclado.
//Controlados es la clase. Ahora necesito crear objetos de esta
//clase:
Controlados controlados1;

#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2

#define interruptON bitSet(SREG,7) //habilita las interrupciones globales
#define interruptOFF bitClear(SREG,7)//desactiva las interrupciones globales
#define pulsador 2

RH_ASK driver;//(2000, 0, 1, 15); //Cambio los pines de rx y tx para usar los del uart
                               //y el pin de enable de 10 a 15
RHReliableDatagram manager(driver, SERVER_ADDRESS);

int modo=0;

void setup() {
  interruptOFF; // se desactivan las interrupciones para configurar.
  //controlados1.configPinesMotores();
  //controlados1.modoStop();
  //controlados1.configTimerMotores(1);//Uso timer=1
  //controlados1.actualizarDutyCycleMotores(50,10); 
  //pinMode(pulsador,INPUT);
  pinMode(LED_BUILTIN,OUTPUT);
  interruptON;//Activo las interrupciones
  Serial.begin(9600);
  if (!manager.init())
    Serial.println("init failed");

}

// Dont put this on the stack:
uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
//char buf[RH_ASK_MAX_MESSAGE_LEN];
//bool estado;

//int aux[RH_ASK_MAX_MESSAGE_LEN];

void loop() {
  if (manager.available())//Miro si recibí algún mensaje
  {
    // Wait for a message addressed to us from the client
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (manager.recvfromAck(buf, &len, &from))
    {
      //Acá recibo el comando del Master. El dato recibido queda
      //en buf (la rutina modifica ese valor).
      //Nota para la ignorancia de Tania: c++ modifica los valores
      //de las variables de entrada de la función.
//      for (int i=0; i <= 59; i++)
//      {
//        aux[i]=(int*) buf[i];
//      }
      //(void) manager.sendtoWait((char*)"Respuesta: ", 11, from);
      //(void) manager.sendtoWait((char*)string(aux), sizeof((char*)string(aux)), from);
      (void) manager.sendtoWait(buf, sizeof(buf), from);
      Serial.print("Recibido: ");
      Serial.println((char*)buf);
      //Serial.println((char*)aux);
    }
  }
}

//int conversion(char palabra[RH_ASK_MAX_MESSAGE_LEN];)
