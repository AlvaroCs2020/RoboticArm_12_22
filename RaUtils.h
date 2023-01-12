#include "Arduino.h"
#include <Servo.h>
#define SERVO1 2 //Base, right 
#define SERVO2 4 //Base, left
#define SERVO3 5 //Arm, first
#define SERVO4 6 //Wrist, first
#define BUTTON 10
#define START 90 //Start angle2
#define I 0
#define J 1
#define K 2
#define RES 0.005
#define MIN_P 500
#define MAX_P 2500
#define OFFSET_S1 150
#define OFFSET_S2 100
#define IO_DELAY 10

//Control of first 3 DOF
void controllArm(int xInput, int yInput, float v_target[], float length0,float length1, Servo sBase,Servo sArm1, Servo sArm2);

//Control of servo activated jaw
int controllJaw(int xInput, Servo sJaw, int jawAperture);