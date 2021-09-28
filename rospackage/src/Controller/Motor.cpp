#include "Motor.h"

Motor::Motor() {}
Motor::~Motor() {}
    
void Motor::init(const int fPin, const int bPin, const int sPin)
{
  forwardPin = fPin;
  backwardPin = bPin;
  speedPin = sPin;

  pinMode(forwardPin, OUTPUT);
  pinMode(backwardPin, OUTPUT);
  pinMode(speedPin, OUTPUT);
}

void Motor::setSpeed(float percentage)
{

  if (fabs(percentage) < 0.001)
  {
    digitalWrite(forwardPin, LOW);
    digitalWrite(backwardPin, LOW);
    analogWrite(speedPin, 0);
  }
  else if (percentage > 0)
  {
    digitalWrite(forwardPin, HIGH);
    digitalWrite(backwardPin, LOW);
    analogWrite(speedPin, convert(percentage));
  }
  else
  {
    digitalWrite(forwardPin, LOW);
    digitalWrite(backwardPin, HIGH);
    
    analogWrite(speedPin, convert(percentage));
  }
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
