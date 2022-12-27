#include <Servo.h>
//(Me salto el pin 3 por que pareciera no funcionar en el arduino uno)
#define SERVO1 2 //Base, right 
#define SERVO2 4 //Base, left
#define SERVO3 5 //Arm, first
#define SERVO4 6 //Wrist, first
#define START 90 //Start angle2
Servo sBase;
Servo sArm1;
Servo sArm2;
Servo sWrist;
int i;
int j;
int x;        // variable para almacenar valor leido del eje X
int y;        // variable para almacenar valor leido del eje y
int z;
int angle0;
int angle1;
int angle2;
void rotateBase();
void rotateArm1();
void rotateArm2();
void setup() 
{
  Serial.begin(9600); // open the serial port at 9600 bps:
  sBase.attach(SERVO1);
  sArm1.attach(SERVO2);
  sArm2.attach(SERVO3);
  sWrist.attach(SERVO4);
  angle1 = START;
  angle2 = START;
  /*
  sArm2.write(START);
  sBase.write(START);
  sBase.write(START);
  */
}

void loop() 
{
  rotateBase();
  rotateArm1();
  rotateArm2();

}
void rotateBase()
{
  z = analogRead(A2);
  delay(10);
  angle0 = map(z,0,1023,0,180);
  sBase.write(180 - angle0);
}
void rotateArm1()
{
  x = analogRead(A0); // lectura de valor de eje x

  if (x >= 0 && x < 480 && 0 < angle1) // si X esta en la zona izquierda
  {
    angle1--;
    delay(25);
  }
  else if (x > 520 && x <= 1023 && angle1 < 180) // si X esta en la zona derecha
  {
    angle1++;
    delay(25);
  }
  sArm1.write(angle1);
} 
void rotateArm2()
{
  Serial.println(angle2);
  y = analogRead(A1);     // lectura de valor de eje x

  if (y >= 0 && y < 480 && 0<angle2)// si X esta en la zona izquierda
  {          
    angle2--;
    delay(25);
  }else  if (y > 520 && y <= 1023 && angle2<180)// si X esta en la zona derecha
  {          
    angle2++;
    delay(25);
  }

  sArm2.write(angle2);
}
