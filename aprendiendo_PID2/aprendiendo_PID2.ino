
#define PIN_INPUT A7
#define PIN_OUTPUT 3

int   setpoint =500;
int Input,Output,Output2,bandera,Output3,error;
float k,kd,a,error2,ki;
void setup()
{
    pinMode(PIN_OUTPUT, OUTPUT);
    pinMode(13, OUTPUT);
  setpoint =800;

  
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
//myPID.SetTunings(Kp, Ki, Kd);

  }
  k=0.1;
  kd=0.01;
  ki=1;
  a=0;
  bandera=0;
}

void loop()
{

  Input = analogRead(PIN_INPUT);

  error=(setpoint-Input);
  
  a=error-error2;
  Output=(int)(k*error+kd*a+ki*Output2);
   error2=error;
  Output2=Output;
   
  if ( ( abs(error) < 50)  && (bandera==0) )
{

 Serial.println("convergio ");
 bandera=1;
 Serial.print(Output3,DEC);
   Serial.print(" ");
    Serial.print(error,DEC);
    Serial.print(" ");
   // Serial.print(error2,DEC);
   // Serial.print(" ");
    Serial.print(a,DEC);
    Serial.print(" ");
      Serial.println(Input,DEC);
      
   delay(100);
   digitalWrite(13,HIGH);
 
}
else if (abs(error) >50) 
{
   
 if (Output >254) {Output=255;}
 if (Output <0) {Output=0;}

  analogWrite(PIN_OUTPUT, Output);
Output3=Output;
  Serial.print(Output2,DEC);
   Serial.print(" ");
    Serial.print(error,DEC);
    Serial.print(" ");
   // Serial.print(error2,DEC);
   // Serial.print(" ");
    Serial.print(a,DEC);
    Serial.print(" ");
      Serial.println(Input,DEC);
      
   
      bandera=0;
      
      delay(100);
      digitalWrite(13,LOW);
  }
//  if (a==0)
//  {
//    k=k+0.001;
//     Serial.println(k,DEC);
//    }

}



