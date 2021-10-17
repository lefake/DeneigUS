#include "Motor.h"

Motor::Motor() {}
Motor::~Motor() {}
    
void Motor::init(const int fPin, const int sPin)
{
  forwardPin = fPin;
  speedPin = sPin;

  pinMode(forwardPin, OUTPUT);
  pinMode(speedPin, OUTPUT);
}

void Motor::setSpeed(float percentage)
{
  digitalWrite(forwardPin, percentage < 0);
  analogWrite(speedPin, convert(percentage));
}

int Motor::convert(float percentage)
{
  float absVal = fabs(percentage);
  return floatMap(absVal, 0, 1, 0, 255);
}

int Motor::floatMap(float x, float inMin, float inMax, float outMin, float outMax)
{
 return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin; 
}
