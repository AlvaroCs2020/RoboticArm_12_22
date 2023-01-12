#include "Arduino.h"
#define RES 0.005
#define MIN_P 500
#define MAX_P 2500
#define OFFSET_S1 150
#define OFFSET_S2 100
#define IO_DELAY 10
void inverseKinematics(float x, float y, float z, int angles[], float length0,float length1);