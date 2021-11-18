#ifndef _ACTUATOR_H
#define _ACTUATOR_H

#include <Arduino.h>
#include "Constants.h"

class Actuator{
  public:
    Actuator();
    ~Actuator();

    void init(const int, const int, const int, const int);
    void enable(int);
    void disable();
    int getPos();

  private:
    int forwardLSPin;
    int backwardLSPin;
    int relay1Pin;
    int relay2Pin;

};

#endif // _MOTOR_H
