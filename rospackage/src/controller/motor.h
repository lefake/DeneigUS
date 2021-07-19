#ifndef MOTOR_H
#define MOTOR_H
#include <Arduino.h>
#include "constants.h"

class Motor
{
  public:
    Motor(int pin_1, int pin_2, int pwm_pin);
    void rotate(float);
    void stop_rotation();  

  private:
    int _forward_pin;
    int _backward_pin;
    int _pwm_pin;
    
};

#endif
