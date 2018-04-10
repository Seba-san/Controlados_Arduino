// ask_receiver.pde
// -*- mode: C++ -*-
// Simple example of how to use RadioHead to receive messages
// with a simple ASK transmitter in a very simple way.
// Implements a simplex (one-way) receiver with an Rx-B1 module

#include <RH_ASK.h>
#include <SPI.h> // Not actualy used but needed to compile

RH_ASK driver;
// RH_ASK driver(2000, 2, 4, 5); // ESP8266 or ESP32: do not use pin 11

void setup()
{
    Serial.begin(9600);	// Debugging only
    if (!driver.init())
         Serial.println("init failed");
}


int n=0;
int k=0;
int tx=1;

void loop()
{
    const char *msg = "Hola, Seba";

//    n=n+1;
//    k=k+1;
//    if (n>1000){
//      n=0;
//      if(k>100){
//        k=0;
      if (tx==1){
        tx=0;
        driver.send((uint8_t *)msg, strlen(msg));
        driver.waitPacketSent();
        Serial.println("Transmitiendo");
      }
//      }
//    }
    
    uint8_t buf[RH_ASK_MAX_MESSAGE_LEN];
    uint8_t buflen = sizeof(buf);

    if (driver.recv(buf, &buflen)) // Non-blocking
    {
	int i;
  tx=1;

	// Message with a good checksum received, dump it.
	driver.printBuffer("Got:", buf, buflen);
  Serial.print("Got:");
  Serial.println((char*)buf);
    }
}
