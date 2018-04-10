//#include "interrupt.h" // Necesario para manipular facilmente  las interrupciones, fue extraido de AtmelSutdio 7

#define interruptON bitSet(SREG,7) //habilita las interrupciones globales
#define interruptOFF bitClear(SREG,7)//desactiva las interrupciones globales

int const OC0A = 6;//10;
int const OC0B = 5;//9;
//int const TXD = 1;//31;
//int const RXD = 0;//30;

void setup() {
  // put your setup code here, to run once:
    interruptOFF; // se desactivan las interrupciones para configurar todo correctamente.
    Serial.begin(9600); // opens serial port, sets data rate to 9600 bps

    pinMode(LED_BUILTIN, OUTPUT); 
    pinMode(OC0A, OUTPUT); //pin 10=OC0A=PD6
    pinMode(OC0B, OUTPUT); //pin 9=OC0B=PD5
//    pinMode(TXD, OUTPUT); //pin 31=TXD=PD1
//    pinMode(RXD, INPUT);//pin 30=RXD=PD0
  // Config Timer 0
  //Obs: el timer0 se usa para las fc millis() y delay(), así que si las necesito tengo que dejar la frec por defecto
    TCCR0A=0b10100011;//Modo fastPWM
    TCCR0B=0b00000101;//Frec=fosc/1024=15625 (creo)
    TIMSK0=0x00;//Sin interrupciones
    TIFR0=0x00;//Borro las banderas
    TCNT0=0x00;//conteo inicia en 0
    OCR0A=30;//Dutycycle para la salida A
    OCR0B=128;//Dutycycle para la salida B (que por ahora está al pedo)
    interruptON;//Activo las interrupciones
    Serial.println(0);
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED_BUILTIN,bitRead(PORTD, 6));
//    bool aux=0;//Variable auxiliar
//    aux=bitRead(PORTD, 6);//Leo PD6=OC0A
//    Serial.println(aux);
//    delay(1000);
//    if (aux=1){
//      digitalWrite(LED_BUILTIN,HIGH);
//    }
//    else {
//      digitalWrite(LED_BUILTIN,LOW);
//    }
}

//void Comunic() { //Interrupción de la comunicación serie
void serialEvent() {
  int dato;
  if (Serial.available() > 0) {
    dato= Serial.read();
    if (dato>=0 && dato<256) {
      OCR0A=dato;//Dutycycle para la salida A (indicado de 0 a 255)
    }
    Serial.println(OCR0A+1);
  }
}
