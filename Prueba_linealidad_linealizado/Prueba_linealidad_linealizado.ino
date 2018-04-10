
/*
   Hace un barrido de PWM y lo envia a matlab para ver como es la respuesta (si es lineal o no). Esto lo hace solo para el estado estacionario.
*/

#define PIN_INPUT A7
#define PIN_OUTPUT 3
#define interruptON bitSet(SREG,7)//habilita las interrupciones 
//globales
#define interruptOFF bitClear(SREG,7)//desactiva las 
//interrupciones globales

int   setpoint;
int Entrada[5], Error[5], Salida[5];
int  bandera;
bool varr;
float kp, kd, ki, delta_error, int_error, ticc, tocc;
// Secuencia 
unsigned char  secuencia[]={2,35,82,83,84,84,85,85,86,86,86,87,87,87,88,88,88,88,89,89,89,89,90,90,90,90,90,91,91,91,91,92,92,92,92,92,93,93,93,93,93,93,94,94,94,94,94,95,95,95,95,95,95,96,96,96,96,96,96,97,97,97,97,97,97,98,98,98,98,98,99,99,99,99,99,99,100,100,100,100,100,100,101,101,101,101,101,102,102,102,102,102,103,103,103,103,103,104,104,104,104,104,105,105,105,105,105,106,106,106,106,106,107,107,107,107,108,108,108,108,109,109,109,109,110,110,110,110,111,111,111,112,112,112,112,113,113,113,113,114,114,114,115,115,115,116,116,116,116,117,117,118,118,118,119,119,119,120,120,121,121,122,122,122,123,123,124,124,125,125,125,126,126,127,128,128,129,129,129,130,131,131,132,132,134,134,135,135,136,137,138,138,139,140,141,141,142,142,143,143,145,146,146,148,148,149,151,151,152,154,154,157,158,160,159,163,162,164,165,167,168,172,170,173,174,175,177,178,180,185,184,186,188,191,193,196,199,202,204,206,208,212,217,216,219,223,230,228,234,239};
int m, l, kk, n;
float A, B, C, D;
int Ts = 500; // tiempo de muestreo en milisegundos

//int error_[3]; // Hay problemas con la lectura del analogread si cargo este vector :/
void setup()
{

  pinMode(PIN_OUTPUT, OUTPUT);
  pinMode(13, OUTPUT);
  //pinMode(PIN_INPUT, INPUT);



  Serial.begin(2000000);
  //while (!Serial) {
  //}
  // analogWrite(PIN_OUTPUT, 0);
  setpoint = 800;
  kp = 0.33; // 0.33 ; Segun zigger nichols, tendrian que ser: kp=0.816, Ti=0.55s , Td=0.1375s (con el metodo oscilatorio)
  kd = -0.1; //-0.1
  ki = 1; // 1

  bandera = 5;

  A = 0; B = 0; C = 0; D = 0;
  interruptON;
 tic();
  n = 0;
  digitalWrite(13, 0);
  m = 5; // Cantidad de muestras antes de cambiar el bit de secuencia
}

void loop() // $1 MAIN LOOP #######################################################################
{
  n = 0; l = 0; kk = 0;
  int largo = sizeof(secuencia);
  //int Entradas[largo * m];
  //unsigned long tiempo[largo * m];
  int Entrada,valor=0;
  //unsigned long tiempo;

digitalWrite(13, 1);
   while (bandera == 5) // Espero a recibir un dato, no importa cual, esto es para reiniciar el test sin tener que recetear.
  {
    float a = Leer_232();
    if (a) {
      bandera = 0;

      Limpiar_buffer();
     
    }
  }
    digitalWrite(13, 0);
    delay(5000);

  while (bandera == 0) // Saco la secuencia y leo la entrada. Una cantidad de "largo" medidas
  {

    analogWrite(PIN_OUTPUT, secuencia[valor]);
     delay(Ts);
      Entrada = analogRead(PIN_INPUT);
       Serial.println(valor);
      // Serial.print(" ; ");
      Serial.println(Entrada);   

        valor=valor+1;
         if (valor > largo-1) {
          bandera=5;
         }  
  }

  digitalWrite(13, 1);
  analogWrite(PIN_OUTPUT, 0);

  while (bandera == 1) // Espero a recibir un dato, no importa cual
  {
    float a = Leer_232();
    if (a) {
      bandera = 2;
      Limpiar_buffer();
    }
  }

}



// $2 Funciones #######################################################################

// ######### Recibe datos por 232 y modifica una variable. Se podria hacer generica.

void Limpiar_buffer() {
  int aa;
  aa = 1;
  while (aa) {
    aa = Leer_232();

  }
}
float Leer_232()
{

  if (Serial.available() > 0) {
    char data;
    // get incoming byte:
    data = Serial.read();
    //Serial.println(data);
    //    if (data>0xA){
    //      k2=k2+0.01;
    //
    //      }
    //      else
    //      {
    //         k2=k2-0.01;
    //        }
    //    }
    //Serial.println(k2,DEC);
    return 1;
  }
  return 0;

}

void Calc_PID ()
{

  // Calculo de parametros
  Error[0] = setpoint - Entrada[0] ;
  delta_error = Error[0] - Error[1];
  int_error = Error[0] + int_error;
  if (int_error > 2400) {
    int_error = 2400;
  }
  if (int_error < -2400) {
    int_error = -2400;
  }

  Salida[0] = (int)(kp * (Error[0] + kd * delta_error + ki * int_error)); //

  //Salida[0]=(int)(A*Salida[1]+B*Error[0]+C*Error[1]+D*error3)

  if (Salida[0] > 254) {
    Salida[0] = 255;
  }
  if (Salida[0] < 0) {
    Salida[0] = 0;
  }

  for (int i = 3; i >= 0; i--) { // Historia de variables; Hay que tener cuidado por como se actualizan, Puede pasar que te queden todos iguales.
    Entrada[i + 1] = Entrada[i];
    Error[i + 1] = Error[i];
    Salida[i + 1] = Salida[i];
  }
}

void Env_salida()
{
  Serial.println (Entrada[0], DEC);
}

void Enviar_estado()
{
  Serial.print ("Salida: ");
  Serial.print(Salida[2], DEC);
  Serial.print(" ");
  Serial.print ("Error: ");
  Serial.print(Error[2], DEC);
  Serial.print(" ");
  Serial.print ("Delta error: ");
  Serial.print(delta_error, DEC);
  Serial.print(" ");
  Serial.print ("integral error: ");
  Serial.print(int_error, DEC);
  Serial.print(" Medicion: ");
  Serial.println(Entrada[0], DEC);

}


// #################### Esta funcion verifica por medio de la variancia si la salida es estable.
bool estable()
{
  float Var, Media;
  int i = 0;
  Media = 0; Var = 0;

  for (i = 0; i <= 4; i++) {
    Media = Media + Entrada[i]; // Entrada[i]
  }
  Media = Media / 5;
  i = 0;
  for ( i = 0; i <= 4; i++) {
    Var = (Media - (float)Entrada[i]) * (Media - (float)Entrada[i]) + Var; // No deja poner ^2
  }
  Var = Var / 5;
  //Serial.println(Var,DEC);
  if (Var < 3) { // valor arbitrario basado en experimentacion;
    return true;
  }
  else
  {
    return false;
  }

}

// ##################### Estas 2 funciones hacen los mismo que en matlab :-)
void tic()

{
  ticc = micros();
}

void toc()
{
  tocc = micros() - ticc;
}




