//#include "interrupt.h" // Necesario para manipular facilmente  las
//interrupciones, fue extraido de AtmelSutdio 7

#include <RHReliableDatagram.h>
#include <RH_ASK.h>
#include <SPI.h>

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

RH_ASK driver(2000, 11, 14, 15); //Cambio el pin de tx al 14 que no lo uso
                               //y el pin de enable de 10 a 15
RHReliableDatagram manager(driver, SERVER_ADDRESS);

int dato;
int aux=1;
int fin=0;
int modo=0;

void setup() {
  interruptOFF; // se desactivan las interrupciones para configurar.
  Serial.begin(9600); // opens serial port, data rate 9600 bps.
  if (!manager.init()) //Esto no sé qué hace, pero parece que es clave.
    Serial.println("init failed");
  controlados1.configPinesMotores();
  controlados1.modoStop();
  controlados1.configTimerMotores(1);//Uso timer=1
  //controlados1.actualizarDutyCycleMotores(50,10); 
  pinMode(pulsador,INPUT);
  pinMode(LED_BUILTIN,OUTPUT);
  interruptON;//Activo las interrupciones
}

uint8_t data[] = "Hello World!";
// Dont put this on the stack:
uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];

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
      
      // Send a reply back to the originator client
      if (!manager.sendtoWait(buf, sizeof(buf), from))
        Serial.println("sendtoWait failed");
    }
  }
}

void serialEvent() {
  int dato;
  if (Serial.available() > 0) {
    dato= Serial.read();
    comandos(dato);
  }
}


void comandos(int dato){
  //Esta función decide qué hacer en función del dato comunicado
  //por la computadora. El modo de comunicación está determinado
  //por el lugar del cuál se obtiene el byte recibido, "dato".
  if (modo==0)//Si estoy en modo iddle estoy disponible para recibir
              //una instrucción.
  {
    switch (dato){
      case 1: //Instrucción 1: adelante
        controlados1.modoStop();
        controlados1.modoAdelante();
        Serial.println("Adelante");
        break;
      case 2: //Instrucción 2: atrás
        controlados1.modoStop();
        controlados1.modoAtras();
        Serial.println("Atras");
        break;
      case 3: //Instrucción 3: stop
        controlados1.modoStop();
        Serial.println("Stop");
        break;
      case 4: //Instrucción 4: cambiar PWM
        modo=1;
        Serial.println("Indicar PWM");
        break;
    }
  }
  else if (modo==1){ //Si modo=1, estoy esperando el nuevo 
                     //valor del PWM
    controlados1.actualizarDutyCycleMotores(dato,dato);//Actualizo los dos PWM
    Serial.println("PWM modificado");
    modo=0;//Vuelvo al modo idle para recibir una nueva instrucción
  }
}

