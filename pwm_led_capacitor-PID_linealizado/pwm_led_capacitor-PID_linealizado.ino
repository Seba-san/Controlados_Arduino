
/*
 * La idea de este programa es encontrar el Kinestable, es decir. Comenzar a aumentar el kp hasta que la salida se vuelva inestable. Esto se logra haciendo una medicion 
 * de la variancia de las 5 ultimas muestras.
 */


#define PID_clasico 1

#if PID_clasico
   #define PIN_INPUT A7//19
  #define PIN_OUTPUT 3//18
#else
  #define PIN_INPUT A7  
  #define PIN_OUTPUT 3
#endif

int Entrada[5],Error[5],Salida[5],setpoint;
int  bandera,data;
bool varr;
unsigned char  secuencia[]={2,2,2,2,2, 2,35,82,83,84,84,85,85,86,86,86,87,87,87,88,88,88,88,89,89,89,89,90,90,90,90,90,91,91,91,91,92,92,92,92,92,93,93,93,93,93,93,94,94,94,94,94,95,95,95,95,95,95,96,96,96,96,96,96,97,97,97,97,97,97,98,98,98,98,98,99,99,99,99,99,99,100,100,100,100,100,100,101,101,101,101,101,102,102,102,102,102,103,103,103,103,103,104,104,104,104,104,105,105,105,105,105,106,106,106,106,106,107,107,107,107,108,108,108,108,109,109,109,109,110,110,110,110,111,111,111,112,112,112,112,113,113,113,113,114,114,114,115,115,115,116,116,116,116,117,117,118,118,118,119,119,119,120,120,121,121,122,122,122,123,123,124,124,125,125,125,126,126,127,128,128,129,129,129,130,131,131,132,132,134,134,135,135,136,137,138,138,139,140,141,141,142,142,143,143,145,146,146,148,148,149,151,151,152,154,154,157,158,160,159,163,162,164,165,167,168,172,170,173,174,175,177,178,180,185,184,186,188,191,193,196,199,202,204,206,208,212,217,216,219,223,230,228,234,239};

#if  PID_clasico
float A,B,C,D,E;
#else
float kp,kd,ki,delta_error,int_error;
#endif

float ticc,tocc;
//int error_[3]; // Hay problemas con la lectura del analogread si cargo este vector :/
void setup()
{
    pinMode(PIN_OUTPUT,OUTPUT);
    //pinMode(13, OUTPUT);
    //pinMode(PIN_INPUT, INPUT);
  

  
  Serial.begin(2000000);
  while (!Serial) {
  }
  
setpoint=500;
 
#if PID_clasico
A=0.742;
B=0.0056;
C=-0.7364;
D=1;
E=0;
#else
 kp=0.33; // 0.33 ; Segun zigger nichols, tendrian que ser: kp=0.816, Ti=0.55s , Td=0.1375s (con el metodo oscilatorio)
 kd=-0.1; //-0.1
 ki=1; // 1
#endif



  
}

void loop() // $1 MAIN LOOP #######################################################################
{
  //Serial.print(" Tiempo: ");
  //Serial.println(tocc,DEC);
  //tic();
  
 Entrada[0] = analogRead(PIN_INPUT);
//  varr=estable();    
  Calc_PID();
 analogWrite(PIN_OUTPUT,secuencia[Salida[0]]);
 Env_salida();
 tic();
 //Leer_232();
  delay(1);
 // toc();

}

// ######### Recibe datos por 232 y modifica una variable. Se podria hacer generica.
void Leer_232()
{
if (Serial.available() > 0) { 
data=Serial.parseInt();
Serial.println(data,DEC);
}
else {
  data=0;
  }

}

void Calc_PID ()//$2
{ 
   for (int i=3; i >= 0; i--){ // Historia de variables; Hay que tener cuidado por como se actualizan, Puede pasar que te queden todos iguales.
    Entrada[i+1]=Entrada[i];
    Error[i+1]=Error[i];
    Salida[i+1]=Salida[i];   
  }
     Error[0]= setpoint-Entrada[0] ; 
  #if PID_clasico
  //Salida[0]=(int)(A*Salida[1]+B*Salida[2]+C*Error[0]+D*Error[1]+E*Error[2]);
  Salida[0]=(int)(D*Salida[1]+E*Salida[2]+A*Error[0]+B*Error[1]+C*Error[2]);
  #else
  // Calculo de parametros 
  delta_error=Error[0]-Error[1];
  int_error=Error[0]+int_error;
  // aplico la ventana 
  if (int_error>2400){int_error=2400;}
  if (int_error<-2400){int_error=-2400;}
  Salida[0]=(int)(kp*(Error[0]+kd*delta_error+ki*int_error)); // 
  #endif
  
  if (Salida[0] >254) {Salida[0]=255;}
  if (Salida[0] <0) {Salida[0]=0;}  

   
}

void Env_salida()
{
   Serial.println (Entrada[0],DEC);
//   toc();
//   Serial.println (tocc,DEC); 
  }

void Enviar_estado()
{
  #if (PID_clasico==0 )
  Serial.print ("Salida: ");
  Serial.print(Salida[2],DEC);
  Serial.print(" ");
    Serial.print ("Error: ");
  Serial.print(Error[2],DEC);
  Serial.print(" ");
   Serial.print ("Delta error: ");
  Serial.print(delta_error,DEC);
   Serial.print(" ");
  Serial.print ("integral error: ");
   Serial.print(int_error,DEC);
  Serial.print(" Medicion: ");
  Serial.println(Entrada[0],DEC);
  #endif
  
}


// #################### Esta funcion verifica por medio de la variancia si la salida es estable.
bool estable()
{
  float Var,Media;
  int i=0;
  Media=0;Var=0;
  
  for (i=0; i <= 4; i++){    
   Media=Media+Entrada[i];  // Entrada[i]
  }
  Media=Media/5;
  i=0;
  for ( i=0; i <= 4; i++){    
   Var=(Media-(float)Entrada[i])*(Media-(float)Entrada[i])+Var;   // No deja poner ^2
  }
  Var=Var/5;
  //Serial.println(Var,DEC);
  if (Var<3){ // valor arbitrario basado en experimentacion;
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
    ticc=micros();
  }

void toc()
  {
   tocc=micros()-ticc;    
    }
  
  
  

