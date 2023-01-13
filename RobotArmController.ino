#include <Servo.h>
#include "SimpleIK.h"
#include "RaUtils.h"
#include "RaBluetooth.h"
//(Me salto el pin 3 por que pareciera no funcionar en el arduino uno)
#define SERVO1 2 //Base, right 
#define SERVO2 4 //Base, left
#define SERVO3 5 //Arm, first
#define SERVO4 6 //Wrist, first
#define BUTTON 10
#define START 90 //Start angle2
#define RES 0.005
#define MIN_P 500
#define MAX_P 2500
#define OFFSET_S1 150
#define OFFSET_S2 100
#define IO_DELAY 10
#define I 0
#define J 1
#define K 2
#define RES_UNITY 0.05
//Commands from Unity
String c_incX = "x+";
String c_decX = "x-";
String c_incY = "y+";
String c_decY = "y-";
String c_incZ = "z+";
String c_decZ = "z-";
String c_sa1 = "s1";
String c_sa2 = "s2";
String c_go1 = "g1";
String c_go2 = "g2";




Servo sBase;
Servo sArm1;
Servo sArm2;
Servo sWrist;

//All units are milimeters (mm)
//Target point, initialized in a known postion

float v_target[] = {0,2,0};

//Lengths
float length0 = 1;
float length1 = 1;
float length2;
bool controllingJaw = false;
int jawAperture = 0;
int x;        // variable para almacenar valor leido del eje X
int y;        // variable para almacenar valor leido del eje y
int z;
void setup() 
{
  Serial.begin(9600); 
  sBase.attach(SERVO1);
  sArm1.attach(SERVO2);
  sArm2.attach(SERVO3);
  pinMode(BUTTON, INPUT);
  //inverseKinematics(v_target[I],v_target[J],v_target[J],angles, length0, length1);
}
void loop() 
{
  y = analogRead(A1);
  x = analogRead(A0);
  
  //--------------Manual mode--------------

  if(digitalRead(BUTTON) == 0)
  {
    controllingJaw = !controllingJaw;
    sWrist.detach();
    delay(300);
  }
  
  
  if(!controllingJaw)
    controllArm(x, y, v_target, length0, length1, sBase, sArm1, sArm2);
  else
    jawAperture = controllJaw( x, sWrist, jawAperture);
  //Serial.println((String) v_target[I] + " " + v_target[J] + " "+v_target[K]);
  //--------------Manual mode--------------
}
void serialEvent()
{
  while(Serial.available() > 0)
  {
    
    String command = Serial.readString();
    

    if(command[0] == c_incX[0] && command[1] == c_incX[1])
    {
      v_target[I]=v_target[I]+RES_UNITY;
    }
    else if(command[0] == c_decX[0] && command[1] == c_decX[1])
    {
      v_target[I]=v_target[I]-RES_UNITY;
    }
    else if(command[0] == c_incY[0] && command[1] == c_incY[1])
    {
      v_target[J]=v_target[J]+RES_UNITY;
    }
    else if(command[0] == c_decY[0] && command[1] == c_decY[1])
    {
      v_target[J]=v_target[J]-RES_UNITY;
    }
    else if(command[0] == c_incZ[0] && command[1] == c_incZ[1])
    {
      v_target[K]=v_target[K]+RES_UNITY;
    }
    else if(command[0] == c_decZ[0] && command[1] == c_decZ[1])
    {
      v_target[K]=v_target[K]-RES_UNITY;
    }
    else if(command[0] == c_sa1[0] && command[1] == c_sa1[1])
    {
      writeEEprom(v_target[I],v_target[J],v_target[K]);
    }
    else if(command[0] == c_go1[0] && command[1] == c_go1[1])
    {
      float point[3];
      int anglesInMicros[3];
      readEEprom(point);
      inverseKinematics(point[I],point[J],point[K],anglesInMicros , length0, length1);
      
      //Serial.println(anglesInMicros[0]);
      //Serial.println(anglesInMicros[1]);
      //Serial.println(anglesInMicros[2]);

      sArm1.writeMicroseconds(anglesInMicros[0]);
      sArm2.writeMicroseconds(anglesInMicros[1]);
      sBase.writeMicroseconds(anglesInMicros[2]);

      delay(3000);

    }
  }
}

