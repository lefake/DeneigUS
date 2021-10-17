#ifndef _MOTOR_H
#define _MOTOR_H

#include <Arduino.h>
#include "Constants.h"

class Motor {
  public:
    Motor();
    ~Motor();

    void init(const int, const int);
    void setSpeed(float);

  private:
    int forwardPin;
    int speedPin;

    int floatMap(float, float, float, float, float);
    int convert(float);
};

#endif // _MOTOR_H
