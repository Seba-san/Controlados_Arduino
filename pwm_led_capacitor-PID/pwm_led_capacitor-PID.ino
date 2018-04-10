
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

int Entrada[5],Error[5],Salida[5],setpoint, Entrada_Actual;
int  bandera,data,u_max,u_min;
bool varr;

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
  
setpoint=150;
u_max=125;
u_min=90;


/*
 * La relacion entre PID_clasico y el no clasico es: 
 * A=kp(kd+ki)
 * B=kp(ki-kd)
 * C=kp*ki
 * En el PID_clasico solo se considera la historia del error hasta el estado 2. En el no clasico se guarda tanto error hasta 2400.
 * Otra diferencia es que en el clasico se considera el valor de la salida pasada. En el no clasico no se considera.
*/ 

/*
 * PID con respuesta escalon (1 polo):  [0.2372    0.0225   -0.2147    1.0000         0]
 * PIDF con respuesta escalon (1 polo):  [3.1321    0.2272   -2.9049    1.0119   -0.0119]
 * PID con PBRS (2 polos): [1.8851   -3.4954    1.6134    1.0000         0]
 * PID con PBRS (1 polo): 0.1179    0.0013   -0.1166    1.0000         0
 *  PID con PBRS (1 polo), muy rapido: 2.2414    0.0291   -2.2123    1.0000         0
 *  PID F, MUY rapido:  2.2194    0.0341   -2.1854    1.0099   -0.0099
 *  5.5737   -5.3230    0.3526    1.0000         0
 *  0.2311    0.0223   -0.2088    1.0000         0
81.3993 -160.1190   78.7418    1.0000         0
80.1618 -157.5851   77.4427    1.0099   -0.0099
0.1901    0.0040   -0.1861    1.0000         0
 */
#if PID_clasico
A= 80.1618 ;//0.2179;//0.297;//0.4379;//13.6681;//0.6685;//0.4231;//0.1626;
B= -157.5851 ;//0.0014;//0.363;//-0.3884;//-5.2973;//-0.4147;//-0.3769;//0.0057;
C=77.4427;//-0.2165;//0.33;//-0.0217;//-0.027;//-0.0021;//-0.021;//-0.1569;
D=1.0099  ;
E=-0.0099;
#else
 kp=0.33; // 0.33 ; Segun zigger nichols, tendrian que ser: kp=0.816, Ti=0.55s , Td=0.1375s (con el metodo oscilatorio)
 kd=-0.1; //-0.1
 ki=1; // 1
#endif


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
 analogWrite(PIN_OUTPUT,(int)Salida[0]);
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
     
  #if PID_clasico
  //Salida[0]=(int)(A*Salida[1]+B*Salida[2]+C*Error[0]+D*Error[1]+E*Error[2]);
  Salida[0]=(D*Salida[1]+E*Salida[2]+A*Error[0]+B*Error[1]+C*Error[2]);
  #else
  // Calculo de parametros 
  delta_error=Error[0]-Error[1];
  int_error=Error[0]+int_error;
  // aplico la ventana 
  if (int_error>2400){int_error=2400;}
  if (int_error<-2400){int_error=-2400;}
  Salida[0]=(int)(kp*(Error[0]+kd*delta_error+ki*int_error)); // 
  #endif
  
  if (Salida[0] >u_max) {Salida[0]=u_max;} // La otra alternativa es que sature la salida, pero la variable la dejas que crezca 
  if (Salida[0] <u_min) {Salida[0]=u_min;}  

   
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
   #if (PID_clasico==1)
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
  
  
  

