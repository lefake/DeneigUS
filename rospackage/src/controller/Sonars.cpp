#include "Sonars.h"

Sonars::Sonars() 
{
  _echo_pins = (int *) malloc(sizeof(int)*NBS_SONARS);
  _trigger_pins = (int *) malloc(sizeof(int)*NBS_SONARS);
}

Sonars::~Sonars()
{ 
  delete[] _echo_pins;
  delete[] _trigger_pins;
}

void Sonars::init(int trigger[], int echo[])
{
  _trigger_pins = trigger;
  _echo_pins = echo;

  for (int i = 0; i < NBS_SONARS; i++)
  {
    pinMode(_trigger_pins[i], OUTPUT);
    pinMode(_echo_pins[i], INPUT);
  }
}

float Sonars::dist(int n)
{
  digitalWrite(_trigger_pins[n], LOW);
  delayMicroseconds(2);
  digitalWrite(_trigger_pins[n], HIGH);
  delayMicroseconds(10);
  digitalWrite(_trigger_pins[n], LOW);
  
  noInterrupts();
  float d = pulseIn(_echo_pins[n], HIGH, pusleTimeout);
  interrupts();

  float distanceM = d / 2.0 * soundSpeed;
  
  if (distanceM <= MIN_DIST_DETECTION_M) 
    return -1.0;
  else if (distanceM > MAX_DIST_DETECTION_M)
    return -2.0 ;
  else
    return distanceM;
}
