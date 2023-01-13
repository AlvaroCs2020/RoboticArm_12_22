#include "Arduino.h"
#include "RaUtils.h"
#include "SimpleIK.h"
int angles[3],
    smoothAngles0,
    smoothAngles1,
    smoothAngles2,
    smoothAngles0Prev,
    smoothAngles1Prev,
    smoothAngles2Prev;

void controllArm(int xInput, int yInput, float v_target[], float length0,float length1, Servo sBase,Servo sArm1, Servo sArm2)
{

  //Input use
  
  if ( xInput < 400 ) 
  {
    v_target[I] =v_target[I]-RES;
    delay(IO_DELAY);
  }
  else if ( xInput > 600 )
  {
    v_target[I]=v_target[I]+RES;
    delay(IO_DELAY);
  }
  
  if ( yInput < 400 )
  {
    v_target[J]=v_target[J]-RES; 
    delay(IO_DELAY);
  }
  else if (yInput > 600)
  {
    v_target[J]=v_target[J]+RES;
    delay(IO_DELAY);
  }
  
  //Simple IK
  
  inverseKinematics(v_target[I],v_target[J],v_target[K],angles, length0, length1);
  
  //Smoothing anlges
  
  smoothAngles0 = angles[0]*0.85 + smoothAngles0Prev *0.15;
  smoothAngles1 = angles[1]*0.85 + smoothAngles1Prev *0.15;
  smoothAngles2 = angles[2]*0.85 + smoothAngles2Prev *0.15;
  
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
  Serial.print(v_target[I]);
  Serial.print(" ");
  Serial.print(v_target[J]);
  Serial.print(" ");
  Serial.print(v_target[K]);
  Serial.print(" (");
  Serial.print(angles[0]);
  Serial.print(";");
  Serial.print(angles[1]);
  Serial.print(";");
  Serial.print(angles[2]);
  Serial.println(")");
  */

  delay(10);
  
}
int controllJaw(int xInput, Servo sJaw, int jawAperture)
{
  sJaw.attach(SERVO4);
  
  if ( xInput < 400 ) 
  {
    jawAperture--;
    delay(IO_DELAY);
  }
  else if ( xInput > 600 )
  {
    jawAperture++;
    delay(IO_DELAY);
  }
  sJaw.write(jawAperture);
  
  return jawAperture;
}

//EEprom handle

void readEEprom(float point[])//int position
{
  Serial.println("Valor almacenado en direccion primera posicion almacenada");	// imprime texto
  point[0] = EEPROM.read(0) /100.0;      		
  point[1] = EEPROM.read(1) /100.0;      		
  point[2] = EEPROM.read(2) /100.0;      		
  
}
void writeEEprom(float v_targetI, float v_targetJ,float v_targetK)
{
  Serial.println("guardamos posicion actual");
  Serial.println((String) v_targetI + " " + v_targetJ + " " +v_targetK);
  EEPROM.write(0, (int)(v_targetI*100));
  EEPROM.write(1, (int)(v_targetJ*100));
  EEPROM.write(2, (int)(v_targetK*100));    
}
