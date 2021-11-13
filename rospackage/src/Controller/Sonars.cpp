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

void Sonars::readPair(int p, FloatArray* msg)
{
  msg->data_count = 3;
  msg->data[0] = p;
  for (int i = 0; i < 2; ++i)
  {
    digitalWrite(triggerPins[p+i], LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPins[p+i], HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPins[p+i], LOW);
    
    noInterrupts();
    float d = pulseIn(echoPins[p+i], HIGH, pusleTimeout);
    interrupts();

    if (d > pusleTimeout)
      d = MAX_DIST_DETECTION_M + 0.5;
    
    msg->data[i+1] = d / 2000000.0 * soundSpeed;  // Test with range min values
  }
}
