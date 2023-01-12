#include "Arduino.h"
#include "SimpleIK.h"
//Article about 2 joint IK: https://www.alanzucconi.com/2018/05/02/ik-2d-2/ 
void inverseKinematics(float x, float y, float z, int angles[], float length0,float length1)
{
  float jointAngle2 = atan(z/x) * (180/PI);
  
  //Rotate point to xy plane
  
  float newX = sqrt(z*z + x*x);
  float newY = y;

  float length2 = sqrt(newX*newX + newY*newY);
  
  float cosAngle0 = ((length2 * length2) + (length0 * length0) - (length1 * length1)) / (2 * length2 * length0);

  float angle0 = acos(cosAngle0) * (180/PI);

  float cosAngle1 = ((length1 * length1) + (length0 * length0) - (length2 * length2)) / (2 * length1 * length0);

  float angle1 = acos(cosAngle1) * (180/PI);
  float atang = atan(newY/newX) * (180/PI);
  float jointAngle0 = atang+angle0 ;// Angle shoulder
  float jointAngle1 = 180.0   - angle1;// Angle elbow
  
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
  
  angles[0] = scaledAngle0 - OFFSET_S1;
  angles[1] = scaledAngle1;
  angles[2] = scaledAngle2;

}