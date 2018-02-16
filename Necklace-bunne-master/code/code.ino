#include <Servo.h>

Servo myservo1;       //initializing object of 1st servo
Servo myservo2;       //initializing object of 2nd servo
Servo myservo3;       //initializing object of 3rd servo
Servo myservo4;       //initializing object of 4th servo


const int ReloadMotor_PotPin = A0;          //Analog pin for the speed control of reload motor
const int Pot_Pin_Delay      = A1;         //Analog pin name where delay pot is connected
const int Pot_Pin_Servo_High = A2;    //Analog pin name where Servo high value pot is connected
const int Pot_Pin_Servo_Low  = A3;     //Analog pin name where Servo low value pot is connected

const int Servo1 = 3;          //Digital PWM pin number where Servo 1st is connected
const int Servo2 = 5;          //Digital PWM pin number where Servo 2nd is connected
const int Servo3 = 6;           //Digital PWM pin number where Servo 3rd is connected
const int Servo4 = 9;           //Digital PWM pin number where Servo 4rd is connected
const int ReloadMotorPin = 11;  //PWM Pin of Reloading Motor

const int Vibrator_ControlPin = 2;        //pin to stop the sequence
const int Servo_SequenceResumeButton = 4;      //pin to resume the sequence

int Servo_StopState = 1;                 //variable to store the statevv of the buttons
int Servo_ResumeState = 0;

int Servo_High=2200;          //variable to keep value of Servo High PWM after mapped from value of Servo_High_Read
int Servo_Low=1500;           //variable to keep value of Servo Low PWM after mapped from value of Servo_Low_Read
int Motor_DutyCycle = 0;      //variable to store duty cycle of the reloading motor

const int Delay_Map_Low=100;        //Minimum value for delay after mapped.
const int Delay_Map_High=5000;      //Maximum value for delay after mapped.
const int Servo_Map_High_High=2200; //Maximum value for Servo High after mapped.
const int Servo_Map_High_Low=1800;  //Minimum value for Servo High after mapped.
const int Servo_Map_Low_High=2100;  //Maximum value for Servo Low after mapped.
const int Servo_Map_Low_Low=1800;   //Minimum value for Servo High after mapped.

int Pot_Value=0;          //variable to keep value from analog pin A1
int Delay_Value=0;        //varibale to keep value of delay after mapped from value of Pot_Value
int Servo_High_Read_Value = 0;
int Servo_Low_Read_Value = 0;
int Motor_Speed_Read_Value = 0;

char Count = 1;

void Custom_Delay( unsigned int delay_value)
{
  unsigned int Loop_Count = delay_value/200;

  while(Loop_Count)
  {
    if(digitalRead(Servo_SequenceResumeButton) && !(Servo_StopState))
      break;
    
    digitalWrite(Vibrator_ControlPin, LOW);
    //CheckState();
    --Loop_Count;
    delay(100);
    digitalWrite(Vibrator_ControlPin, HIGH);
    delay(100);
  }
}

char Sequence_Servo(int Servo_High,int Servo_Low,int Delay_Value, char count) {
  /*
  myservo1.writeMicroseconds(Servo_High);
  delay(15);
  myservo2.writeMicroseconds(Servo_High);
  delay(15);
  delay(500);
  myservo3.writeMicroseconds(Servo_Low);
  delay(15);
  
  delay(Delay_Value);
  Serial.println("1");
  
  myservo3.writeMicroseconds(Servo_High);
  delay(15);
  myservo2.writeMicroseconds(Servo_High);
  delay(15);
  delay(500);
  myservo1.writeMicroseconds(Servo_Low);
  delay(15);
  
  delay(Delay_Value);
  Serial.println("2");
  
  myservo1.writeMicroseconds(Servo_High);
  delay(15);
  myservo3.writeMicroseconds(Servo_High);
  delay(15);
  delay(500);
  myservo2.writeMicroseconds(Servo_Low);
  delay(15);
  
  delay(Delay_Value);
  Serial.println("3");
  */

  switch(count){
    case 1:
    //case 3:
      myservo2.writeMicroseconds(Servo_High);
      myservo3.writeMicroseconds(Servo_High);
      myservo4.writeMicroseconds(Servo_High);
      //digitalWrite(Vibrator_ControlPin, LOW);
      delay(150);  
      myservo1.writeMicroseconds(Servo_Low);
     
      Serial.println("1");
      Custom_Delay(1.5*Delay_Value);
      
      break;

    case 2:
    //case 4:
      myservo1.writeMicroseconds(Servo_High);     
      myservo3.writeMicroseconds(Servo_High);
      myservo4.writeMicroseconds(Servo_High);
      //digitalWrite(Vibrator_ControlPin, HIGH);
      delay(150);
      myservo2.writeMicroseconds(Servo_Low);
      Serial.println("2");
      Custom_Delay(1.5*Delay_Value);
     
      break;
      
    //case 5:
    case 3:
      myservo1.writeMicroseconds(Servo_High);
      myservo2.writeMicroseconds(Servo_High);     
      myservo4.writeMicroseconds(Servo_High);
      delay(150);
      myservo3.writeMicroseconds(Servo_Low);
      Serial.println("3");
      Custom_Delay(1.5*Delay_Value);
     
      break;

    //case 6:
     // myservo1.writeMicroseconds(Servo_High);
      //myservo2.writeMicroseconds(Servo_High);
      //myservo3.writeMicroseconds(Servo_High);
      //delay(150);
      //myservo4.writeMicroseconds(Servo_Low);
      //Serial.println("4");
     
      
      //break;

    default:
      count = 0;

 }
  ++count;
 
  
  return count;
}

void StopStateCompute(void)
{
    //Serial.println("Stopped Condition");
    myservo1.writeMicroseconds(Servo_Low);
    myservo2.writeMicroseconds(Servo_Low);
    myservo3.writeMicroseconds(Servo_Low);
    myservo4.writeMicroseconds(Servo_Low);
    analogWrite(ReloadMotorPin,0);
    digitalWrite(Vibrator_ControlPin, LOW);
}

void CheckState(void)
{
  if(digitalRead(Servo_SequenceResumeButton) && !(Servo_StopState))
  {
    Servo_StopState = 1;
    Servo_ResumeState = 0;
    StopStateCompute();
    Serial.println("Stopped");
        
  }
  else if(!(digitalRead(Servo_SequenceResumeButton) || Servo_ResumeState))
  {
    Servo_StopState = 0;
    Servo_ResumeState = 1;
    myservo1.writeMicroseconds(Servo_High);
    myservo2.writeMicroseconds(Servo_High);
    myservo3.writeMicroseconds(Servo_High);
    myservo4.writeMicroseconds(Servo_High);
    Serial.println("Resume");
  }
}

void setup() {
  
  Serial.begin(9600);           //Serial initialization for debug 
  
  myservo1.attach(Servo1);      //Servo 1st initialized
  myservo2.attach(Servo2);      //Servo 2nd initialized
  myservo3.attach(Servo3);      //Servo 3rd initialized
  myservo4.attach(Servo4);      //Servo 4th initialized
  pinMode(Vibrator_ControlPin, OUTPUT);
  digitalWrite(Vibrator_ControlPin, LOW);
  
  Servo_High_Read_Value=analogRead(Pot_Pin_Servo_High);                                 //Pot value is read from Servo High
  Servo_High=map(Servo_High_Read_Value,0,1023,Servo_Map_High_Low,Servo_Map_High_High);  //Pot value is mapped to a real PWM value of Servo High from 1950 ms to 2100 ms
    
  Servo_Low_Read_Value=analogRead(Pot_Pin_Servo_Low);                                 //Pot value is read from Servo Low
  Servo_Low=map(Servo_Low_Read_Value,0,1023,Servo_Map_Low_Low,Servo_Map_Low_High);    //Pot value is mapped to a real PWM value of Servo Low from 1900 ms to 2100 ms

   
  if(Servo_Low > Servo_High)
    Servo_Low = Servo_High;
  else if(Servo_High < Servo_Low)
    Servo_High = Servo_Low;
  
  StopStateCompute();

  
  
  
  Serial.println("hello");     //to check if serial communication is working or not

} 

void loop() {

   CheckState();
  if(Servo_StopState)
  {
   
    Count = 1;
  }
    
  
  else if(Servo_ResumeState)
  {
    //Serial.println("Run Condition");
    Pot_Value=analogRead(Pot_Pin_Delay);                                //Pot value is read from Delay pot
    Delay_Value=map(Pot_Value,0,1023,Delay_Map_Low,Delay_Map_High);     //Pot value is mapped to a real Delay of 500 ms to 5000 ms.
    
    Motor_Speed_Read_Value = analogRead(ReloadMotor_PotPin);          //Pot analog value for dutycycle of the reloading motor
    Motor_DutyCycle = map(Motor_Speed_Read_Value, 0, 1023, 0, 255); 
    
    Servo_High_Read_Value=analogRead(Pot_Pin_Servo_High);                                 //Pot value is read from Servo High
    Servo_High=map(Servo_High_Read_Value,0,1023,Servo_Map_High_Low,Servo_Map_High_High);  //Pot value is mapped to a real PWM value of Servo High from 1950 ms to 2100 ms
    
    Servo_Low_Read_Value=analogRead(Pot_Pin_Servo_Low);                                 //Pot value is read from Servo Low
    Servo_Low=map(Servo_Low_Read_Value,0,1023,Servo_Map_Low_Low,Servo_Map_Low_High);    //Pot value is mapped to a real PWM value of Servo Low from 1900 ms to 2100 ms

   
    if(Servo_Low > Servo_High)
      Servo_Low = Servo_High;

    else if(Servo_High < Servo_Low)
      Servo_High = Servo_Low;

    
    //check in serial system.
   /* Serial.println("............................................................................");
    Serial.println("............................................................................");
    Serial.println("............................................................................");*/

    Serial.print(Motor_DutyCycle);
    Serial.print(" ");
    Serial.print(Delay_Value);
    Serial.print(" ");
    Serial.print(Servo_High);
    Serial.print(" ");
    Serial.println(Servo_Low);
    
   
    //Serial.println(Servo_StopState);
    //Serial.println(Servo_ResumeState);
   /* Serial.println("............................................................................");

    Serial.println(Motor_Speed_Read_Value); 
    Serial.println(Pot_Value);
    Serial.println(Servo_High_Read_Value);   
    Serial.println(Servo_Low_Read_Value);
    
   
    Serial.println("............................................................................");
    Serial.println("............................................................................");
    Serial.println("............................................................................");*/
    
    
    Count = Sequence_Servo(Servo_High,Servo_Low,Delay_Value, Count);     //sequencing of servo.
    analogWrite(ReloadMotorPin, Motor_DutyCycle);
        //Pwm of Reloading Motor
    
  }
    
}
