#include <Servo.h>
//(Me salto el pin 3 por que pareciera no funcionar en el arduino uno)
#define SERVO1 2 //Base, right 
#define SERVO2 4 //Base, left
#define SERVO3 5 //Arm, first
#define SERVO4 6 //Wrist, first
#define START 90 //Start angle2
#define RES 0.01
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
//Longitudes
float length0 = 1;
float length1 = 1;
float length2;
//Return angles
float angles[2];


int x;        // variable para almacenar valor leido del eje X
int y;        // variable para almacenar valor leido del eje y
int z;

int angle0;
int angle1;
int angle2;
void rotateBase();
void rotateArm1();
void rotateArm2();
void inverseKinematicsB(float x, float y, float angles[]);
void inverseKinematics(float i, float j, float angles[]);
void setup() 
{
  Serial.begin(9600); // open the serial port at 9600 bps:
  sBase.attach(SERVO1);
  sArm1.attach(SERVO2);
  sArm2.attach(SERVO3);
  sWrist.attach(SERVO4);
  sWrist.write(90);
  inverseKinematics(i,j,angles);
  Serial.print("(");
  Serial.print(angles[0]);
  Serial.print(";");
  Serial.print(angles[1]);
  Serial.println(")");
  
}
void loop() 
{
  /*
  delay(50);
  rotateArm1();
  rotateArm2();
  
  //j = 20;
  
  sArm1.write(angles[0]+90);
  sArm2.write(angles[1]);
  
  delay(20);
  */
  rotateBase();
  sWrist.write(0);
  
  x = analogRead(A0); // lectura de valor de eje x
  if (x >= 0 && x < 480 ) 
    i=i-RES;
  else if (x > 520 && x <= 1023 )
    i=i+RES;
  
  y = analogRead(A1); // lectura de valor de eje x
  if (y >= 0 && y < 480 && 0<j )
    j=j-RES; 
  else if (y > 520 && y <= 1023 )
    j=j+RES;
  
  inverseKinematics(i,j,angles);
  sArm1.write(angles[0]);
  sArm2.write(angles[1]);
  Serial.print("punto ");
  Serial.print(i);
  Serial.print(" ");
  Serial.print(j);
  Serial.print(" (");
  Serial.print(angles[0]);
  Serial.print(";");
  Serial.print(angles[1]);
  Serial.println(")");
  delay(50);
}
void inverseKinematics(float x, float y, float angles[])
{
  //Distance from origin to target poin
  length2 = sqrt(x*x + y*y);

  float cosAngle0 = ((length2 * length2) + (length0 * length0) - (length1 * length1)) / (2 * length2 * length0);

  float angle0 = acos(cosAngle0) * (180/PI);

  float cosAngle1 = ((length1 * length1) + (length0 * length0) - (length2 * length2)) / (2 * length1 * length0);

  float angle1 = acos(cosAngle1) * (180/PI);
  float atang = atan(y/x) * (180/PI);//revisar valor d
  float jointAngle0 = atang+angle0 ;// Angle A
  float jointAngle1 = 180.0   - angle1;    // Angle B
  if(i<0)
    jointAngle0= abs(180+jointAngle0);
  angles[0] = jointAngle0;
  angles[1] = jointAngle1;

}

void rotateBase()
{ 
  
  z = analogRead(A2);
  angle0 = map(z,0,1023,0,40);
  int fix = angle0 /10;
  fix *= 10; 
  sBase.write(fix);

}
void rotateArm1()
{
  x = analogRead(A0); // lectura de valor de eje x

  if (x >= 0 && x < 480 && 0 < angle1) // si X esta en la zona izquierda
  {
    angle1--;

  }
  else if (x > 520 && x <= 1023 && angle1 < 180) // si X esta en la zona derecha
  {
    angle1++;
    //delay(25);
  }
  sArm1.write(angle1);
} 
void rotateArm2()
{
  y = analogRead(A1);     // lectura de valor de eje x

  if (y >= 0 && y < 480 && 0<angle2)// si X esta en la zona izquierda
  {          
    angle2--;
    //delay(25);
  }else  if (y > 520 && y <= 1023 && angle2<180)// si X esta en la zona derecha
  {          
    angle2++;
    //delay(25);
  }

  sArm2.write(angle2);
}
