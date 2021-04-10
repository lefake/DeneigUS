#include "Motor.h"

Motor::Motor() {}
Motor::~Motor() {}
    
void Motor::init(const int forward_pin, const int backward_pin, const int speed_pin)
{
  _forward_pin = forward_pin;
  _backward_pin = backward_pin;
  _speed_pin = speed_pin;

  pinMode(_forward_pin, OUTPUT);
  pinMode(_backward_pin, OUTPUT);
  pinMode(_speed_pin, OUTPUT);
}

void Motor::set_speed(float percentage)
{

  if (fabs(percentage) < 0.001)
  {
    digitalWrite(_forward_pin, LOW);
    digitalWrite(_backward_pin, LOW);
    analogWrite(_speed_pin, 0);
  }
  else if (percentage > 0)
  {
    digitalWrite(_forward_pin, HIGH);
    digitalWrite(_backward_pin, LOW);
    
    analogWrite(_speed_pin, convert(percentage));
  }
  else
  {
    digitalWrite(_forward_pin, LOW);
    digitalWrite(_backward_pin, HIGH);
    
    analogWrite(_speed_pin, convert(percentage));
  }
}

int Motor::convert(float percentage)
{
  float abs_val = fabs(percentage);
  return float_map(abs_val, 0, CALC(1.0, 1.0), 0, 255);
}

int Motor::float_map(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min; 
}
