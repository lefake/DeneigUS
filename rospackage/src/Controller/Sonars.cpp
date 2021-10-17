#include "Sonars.h"

Sonars::Sonars() 
{
  echoPins = (int *) malloc(sizeof(int)*NBS_SONARS);
  triggerPins = (int *) malloc(sizeof(int)*NBS_SONARS);
}

Sonars::~Sonars()
{ 
  delete[] echoPins;
  delete[] triggerPins;
}

void Sonars::init(int trigger[], int echo[])
{
  triggerPins = trigger;
  echoPins = echo;

  for (int i = 0; i < NBS_SONARS; i++)
  {
    pinMode(triggerPins[i], OUTPUT);
    pinMode(echoPins[i], INPUT);
  }
}

float Sonars::dist(int n)
{
  digitalWrite(triggerPins[n], LOW);
  delayMicroseconds(2);
  digitalWrite(triggerPins[n], HIGH);
  delayMicroseconds(10);
  digitalWrite(triggerPins[n], LOW);
  
  noInterrupts();
  float d = pulseIn(echoPins[n], HIGH, pusleTimeout);
  interrupts();

  float distanceM = d / 2000000.0 * soundSpeed;
  
  if (distanceM <= MIN_DIST_DETECTION_M) 
    return -1.0;
  else if (distanceM > MAX_DIST_DETECTION_M)
    return -2.0 ;
  else
    return distanceM;
}
