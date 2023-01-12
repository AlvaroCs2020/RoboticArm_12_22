#include <Servo.h>
#include "SimpleIK.h"
#include "RaUtils.h"
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


Servo sBase;
Servo sArm1;
Servo sArm2;
Servo sWrist;

//All units are milimeters (mm)
//Target point, initialized in a known postion

float v_target[] = {1,1,1};

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
  sWrist.attach(SERVO4);
  sWrist.write(0);
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
  //--------------Manual mode--------------
}


