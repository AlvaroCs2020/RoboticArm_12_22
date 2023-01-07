#include <Servo.h>
//(Me salto el pin 3 por que pareciera no funcionar en el arduino uno)
#define SERVO1 2 //Base, right 
#define SERVO2 4 //Base, left
#define SERVO3 5 //Arm, first
#define SERVO4 6 //Wrist, first
#define START 90 //Start angle2
#define RES 0.01
#define MIN_P 500
#define MAX_P 2500
Servo sBase;
Servo sArm1;
Servo sArm2;
Servo sWrist;

//Inverse Kinematics
//Article about 2 joint IK: https://www.alanzucconi.com/2018/05/02/ik-2d-2/ 
//all units are milimeters (mm)

//Target point
float i = 1;
float j = 1;
float k = 1;

//Lengths
float length0 = 1;
float length1 = 1;
float length2;
//Return angles
int angles[3],
    smoothAngles0,
    smoothAngles1,
    smoothAngles2,
    smoothAngles0Prev,
    smoothAngles1Prev,
    smoothAngles2Prev;

bool controllingJaw = false;


int x;        // variable para almacenar valor leido del eje X
int y;        // variable para almacenar valor leido del eje y
int z;

int angle0;
int angle1;
int angle2;
void inverseKinematics(float x, float y, float z,int angles[]);
void manualMode();
void setup() 
{
  Serial.begin(9600); 
  sBase.attach(SERVO1);
  sArm1.attach(SERVO2);
  sArm2.attach(SERVO3);
  sWrist.attach(SERVO4);
  inverseKinematics(i,j,k,angles);
}
void loop() 
{
  
  y = analogRead(A1);
  x = analogRead(A0);
  if(!controllingJaw)
  {
    manualMode();
  }
  else
  {
    //meter la garra la concha de mi hermana
  }
}

void inverseKinematics(float x, float y, float z, int angles[])
{
  //IK calculations

  float jointAngle2 = atan(z/x) * (180/PI);
  //Rotate point to xy plane
  float newX = sin(jointAngle2)*x + cos(jointAngle2)*y;
  float newY = y;
  length2 = sqrt(newX*newX + newY*newY);
  
  float cosAngle0 = ((length2 * length2) + (length0 * length0) - (length1 * length1)) / (2 * length2 * length0);

  float angle0 = acos(cosAngle0) * (180/PI);

  float cosAngle1 = ((length1 * length1) + (length0 * length0) - (length2 * length2)) / (2 * length1 * length0);

  float angle1 = acos(cosAngle1) * (180/PI);
  float atang = atan(newY/newX) * (180/PI);
  float jointAngle0 = atang+angle0 ;// Angle A
  float jointAngle1 = 180.0   - angle1;// Angle B
  
  
  
  if(newX<0)
    jointAngle0= abs(180+jointAngle0);
  
  //Out of range check
  
  if(length0+length1 <length2)
  {
    jointAngle0 = atang;
    jointAngle1 = 0;
  }
  
  //Convert angles to Micros
  
  int scaledAngle0 = (int) (jointAngle0*100);
  int scaledAngle1 = (int) (jointAngle1*100);
  int scaledAngle2 = (int) (jointAngle2*100);
  
  scaledAngle0 = map(scaledAngle0,0,18000,MIN_P,MAX_P);
  scaledAngle1 = map(scaledAngle1,0,18000,MIN_P,MAX_P);
  scaledAngle2 = map(scaledAngle2,0,18000,MIN_P,MAX_P);

  //Return
  
  angles[0] = scaledAngle0;
  angles[1] = scaledAngle1;
  angles[2] = scaledAngle2;
  
  //Debug stuff

  Serial.print("punto ");
  Serial.print(newX);
  Serial.print(" ");
  Serial.print(newY);
  Serial.print(" ");
  Serial.print(z);
  Serial.print(" (");
  Serial.print(jointAngle0);
  Serial.print(";");
  Serial.print(jointAngle1);
  Serial.print(";");
  Serial.print(jointAngle2);
  Serial.println(")");

}
void manualMode()
{
  //Input use
  
  if (x >= 0 && x < 480 ) 
    i=i-RES;
  else if (x > 520 && x <= 1023 )
    i=i+RES;
  
  if (y >= 0 && y < 480 )
    j=j-RES; 
  else if (y > 520 && y <= 1023 )
    j=j+RES;
  
  //Simple IK
  
  inverseKinematics(i,j,k,angles);
  
  //Smoothing anlges
  
  smoothAngles0 = angles[0]*0.9 + smoothAngles0Prev *0.1;
  smoothAngles1 = angles[1]*0.9 + smoothAngles1Prev *0.1;
  smoothAngles2 = angles[2]*0.9 + smoothAngles2Prev *0.1;
  
  smoothAngles0Prev = smoothAngles0;
  smoothAngles1Prev = smoothAngles1;
  smoothAngles2Prev = smoothAngles2;

  //Move the arm
  
  sArm1.writeMicroseconds(smoothAngles0);
  sArm2.writeMicroseconds(smoothAngles1);
  sBase.writeMicroseconds(smoothAngles2);
  /*
  //Debug stuff
  Serial.print("punto ");
  Serial.print(i);
  Serial.print(" ");
  Serial.print(j);
  Serial.print(" ");
  Serial.print(k);
  Serial.print(" (");
  Serial.print(angles[0]);
  Serial.print(";");
  Serial.print(angles[1]);
  Serial.print(";");
  Serial.print(angles[2]);
  Serial.println(")");
  */
  delay(100);
}
