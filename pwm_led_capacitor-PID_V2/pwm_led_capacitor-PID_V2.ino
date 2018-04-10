
  #define PIN_INPUT A7//19
  #define PIN_OUTPUT 3//18


int Entrada[5],Error[5],Salida[5],setpoint, Entrada_Actual;
int  bandera,data,u_max,u_min,offset;
bool varr;
float coeficientes[]={0.261375, -0.421875, 0.164707, 1.886037, -0.886037};


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
  
setpoint=700;
u_max=255;
u_min=0;
offset=0; // offset de salida, hay que sumarle esto a la salida calculada

tic();
  
}

void loop() // $1 MAIN LOOP #######################################################################
{
  //Serial.print(" Tiempo: ");
  //Serial.println(tocc,DEC);
  //tic();
  
 Entrada_Actual = analogRead(PIN_INPUT);
//  varr=estable();    
  Calc_PID();
 analogWrite(PIN_OUTPUT,(int)(Salida[0]+offset));
 Env_salida();
//Enviar_estado();
 //Leer_232();
  delay(10);
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
   for (int i=3; i >= 0; i--){ // Historia de variables; Hay que tener cuidado por como se actualizan, Puede pasar que te queden todos iguales. Solo actualiza los valores pasados, esto es el valor en 0 queda igual al valor en 1, al 0 hay que actualizarlo manualmente.
    Entrada[i+1]=Entrada[i];
    Error[i+1]=Error[i];
    Salida[i+1]=Salida[i];   
  }
    Entrada[0]=Entrada_Actual;
     Error[0]= (setpoint-Entrada[0]); 
  
  //Salida[0]=(int)(A*Salida[1]+B*Salida[2]+C*Error[0]+D*Error[1]+E*Error[2]);
  Salida[0]=(int)(coeficientes[3]*Salida[1]+coeficientes[4]*Salida[2]+coeficientes[0]*Error[0]+coeficientes[1]*Error[1]+coeficientes[2]*Error[2]);
 
  if (Salida[0] > u_max) {Salida[0]=u_max;} // La otra alternativa es que sature la salida, pero la variable la dejas que crezca 
  if (Salida[0] < u_min) {Salida[0]=u_min;}  

   
}

void Env_salida()
{
//  int kaka;
//  kaka=map(Entrada[0],0,1023,0,5000);
//   Serial.println (kaka,DEC);
   Serial.println (Entrada[0],DEC);
  }

void Enviar_estado()
{
  
   
   Serial.print ("Salida: ");
  Serial.print(Salida[0],DEC);
  Serial.print(" ");
    Serial.print ("Error: ");
  Serial.print(Error[0],DEC);
  Serial.print(" ");
   Serial.print ("Delta error: ");
  Serial.print(Error[0]-Error[1],DEC);
   Serial.print(" ");
  Serial.print ("integral error: ");
   Serial.print(Error[0]+Error[1]+Error[2],DEC);
  Serial.print(" Medicion: ");
  Serial.print(Entrada[0],DEC);
  Serial.print(" Correcion: ");
  Serial.println(Salida[0]-Salida[1],DEC);

  
  
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
  
  
  

