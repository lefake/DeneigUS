#include "motor.h"


Motor::Motor(int pin_1, int pin_2, int pwm_pin)
{
  _forward_pin = pin_1;
  _backward_pin = pin_2;
  _pwm_pin = pwm_pin;
  stop_rotation();

  // Motors setup:
  pinMode(_forward_pin,  OUTPUT);
  pinMode(_backward_pin,  OUTPUT);
  pinMode(_pwm_pin, OUTPUT);
}
    
void Motor::rotate(float rotation_speed)
{ 
   
  int _pwm = map(abs(rotation_speed), 0, 1, 0 , 255);
      
  if (rotation_speed < 0){
   // rotates backward
   digitalWrite(_forward_pin, LOW);
   digitalWrite(_backward_pin, HIGH);
   analogWrite(_pwm_pin, _pwm);
  } 
  
  else {
    // rotates forward
    digitalWrite(_forward_pin, HIGH);
    digitalWrite(_backward_pin, LOW);
    analogWrite(_pwm_pin, _pwm);
  }
}

void Motor::stop_rotation()
{
  digitalWrite(_forward_pin, LOW);
  digitalWrite(_backward_pin, LOW);
  analogWrite(_pwm_pin, 0);
}
