//(Me salto el pin 3 por que pareciera no funcionar en el arduino uno)
#define SERVOB_R 2 //Base, right 
#define SERVOB_L 4 //Base, left
#define SERVOA_1 5 //Arm, first
#define SERVOW_1 6 //Wrist, first

#define START 90
#include <Servo.h>
Servo sBaseR;
Servo sBaseL;
Servo sArm;
Servo sWrist;
int currentR;
int currentL;
int currentA;
int moveToB;
int i;
int j;
int x;        // variable para almacenar valor leido del eje X
int y;        // variable para almacenar valor leido del eje y
int angle;
void rotateBaseLink();
void setup() 
{
  Serial.begin(9600); // open the serial port at 9600 bps:
  sBaseR.attach(SERVOB_R);
  sBaseL.attach(SERVOB_L);
  sArm.attach(SERVOA_1);
  sWrist.attach(SERVOW_1);
  sArm.write(START);
  sBaseR.write(START);
  sBaseR.write(START);
  moveToB = START;
  angle=START;
}

void loop() 
{ 
  rotateBaseLink();
  x = analogRead(A0);      // lectura de valor de eje x
  
  if (x >= 0 && x < 480 && 0<moveToB)// si X esta en la zona izquierda
  {          
    moveToB--;
    delay(25);
  }else  if (x > 520 && x <= 1023 && moveToB<180)// si X esta en la zona derecha
  {          
    moveToB++;
    delay(25);
  }
  sBaseR.write(moveToB);
  sBaseL.write(180-moveToB);
  
  /*
  if(moveToB >= 90)
  {
    currentR = sBaseR.read();
    currentL = sBaseL.read();
    for(i = 0 ;i < moveToB-START; i++) //Movemos el primer link
    {
      sBaseR.write(currentR + i);
      sBaseL.write(currentL - i);
      delay(10);
    }
  }
  else
  {
    currentR = sBaseR.read();
    currentL = sBaseL.read();
    for(i = 0 ;i < START-moveToB; i++) //Movemos el primer link
    {
      sBaseR.write(currentR - i);
      sBaseL.write(currentL + i);
      delay(10);
    }
  
  }
  */

}
  /*
  for(j = 0 ;j< angle; j++) //movemos el segundo
  {
    sArm.write(currentA + j);
    delay(10);
  }
  sBaseL.write(0);
  //sArm.write(0);
  //sWrist.write(0);
  rotateFstLink(90);
  //sArm.write(90);
  //sWrist.write(90);
  delay(1500);
  servoMotor.write(0);
  servoMotor2.write(0);
    // Esperamos 1 segundo
  servoMotor.write(180);
    // Esperamos 1 segundo
  if(digitalRead(LEFT) == HIGH)
  {
    servoMotor.write(0);
    // Esperamos 1 segundo
    delay(1500);
    Serial.println(digitalRead(LEFT));
  }
  else if(digitalRead(RIGHT) == HIGH)
  {
    servoMotor.write(90);
    // Esperamos 1 segundo
    delay(1500);
    Serial.println(digitalRead(RIGHT));
  }

  while(digitalRead(LEFT) == LOW){}
  Serial.println(digitalRead(LEFT));
  digitalWrite(STEP, HIGH);       // nivel alto
  delay(10);          // por 10 mseg
  digitalWrite(STEP, LOW);        // nivel bajo
  delay(10);          // por 10 mseg
  while(digitalRead(LEFT) == HIGH){}
  */
void rotateBaseLink()
{
  Serial.println(angle);
  y = analogRead(A1);     // lectura de valor de eje x

  if (y >= 0 && y < 480 && 0<angle)// si X esta en la zona izquierda
  {          
    angle--;
    delay(25);
  }else  if (y > 520 && y <= 1023 && angle<180)// si X esta en la zona derecha
  {          
    angle++;
    delay(25);
  }

  sArm.write(angle);
}
