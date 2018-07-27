#include <Servo.h>

// Define the motor ESC pins
#define THRUSTER_LEFT_STR  2
#define THRUSTER_RIGHT_STR 5
#define THRUSTER_LEFT_ANG  4
#define THRUSTER_RIGHT_ANG 3

Servo TLS; //Thruster on Left pointing Straight
Servo TLA; //Thruster on Left Angled 45
Servo TRS; //Thruster on Right pointing Straight
Servo TRA; //Thruster on Right Angled 45

void writeToMotors(long leftStr, long rtStr, long leftAng, long rtAng)
{
  TLS.writeMicroseconds(leftStr);
  TRS.writeMicroseconds(rtStr);
  TLA.writeMicroseconds(leftAng);
  TRA.writeMicroseconds(rtAng);
}

void setup()
{
  Serial.begin(9600);
  
  Serial.println("Start!");
  
  TLS.attach(THRUSTER_LEFT_STR);
  TLA.attach(THRUSTER_LEFT_ANG);
  TRS.attach(THRUSTER_RIGHT_STR);
  TRA.attach(THRUSTER_RIGHT_ANG);
}

int servoVal = 100; 

void loop() 
{
  char c; 
  if (Serial.available())
  { 
    c = Serial.read();
    Serial.print(c);
    Serial.print(": ");
    Serial.println(servoVal);
  }
 
  switch(c)
  {
    //Keys to test individual motors
    case '1': //TLS
      writeToMotors(servoVal,1500,1500,1500);
      break;
    case '2': //TRS
      writeToMotors(1500,servoVal,1500,1500);
      break;
    case '3': //TLA
      writeToMotors(1500,1500,servoVal,1500);
      break;
    case '4': //TRA
      writeToMotors(1500,1500,1500,servoVal);
      break;
    case '9':
      servoVal = servoVal == 500 ? 500 : servoVal + 50;
      break;
    case '8':
      servoVal = servoVal == 0 ? 0 : servoVal - 50;
      break;
    //STOP
    case '0':
      writeToMotors(1500,1500,1500,1500);
      break;
      
    //Keys to test maneuvers
    
    case 'a': //Yaw CCW
      writeToMotors(1500 + servoVal,1500 + servoVal,1500,1500);
      break;
    case 'w': //Go forwards
      writeToMotors(1500 - servoVal,1500 + servoVal,1500,1500);
      break;
    case 'd': //Yaw CWk
      writeToMotors(1500 - servoVal,1500 - servoVal,1500,1500);
      break; 
    case 's': //Go backwards
      writeToMotors(1500 + servoVal,1500 - servoVal,1500,1500);
      break;
    case 'j': //strafe right
      writeToMotors(1500,1500,1500 + servoVal,1500 - servoVal);
      break;
    case 'k': //descend
      writeToMotors(1500,1500,1500 + servoVal,1500 + servoVal);
      break;
    case 'l': //strafe left
      writeToMotors(1500,1500,1500 - servoVal,1500 + servoVal);
      break;
    case 'i': //asscend
      writeToMotors(1500,1500,1500 - servoVal,1500 - servoVal);
      break;    
  } 
}
    
