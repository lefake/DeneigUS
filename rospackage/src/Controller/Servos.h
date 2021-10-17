#ifndef _SERVOS_H
#define _SERVOS_H

#include <Arduino.h>
#include "Constants.h"
#include <Servo.h>

enum SERVOS 
{
  ELEVATION = 0,
  ROTATION,
};

class Servos {
  public:
    Servos();
    ~Servos();

    void init(int pins[]);
    void setPos(int n, int pos);
    int getPos(int n);

  private:
    Servo servosArray[NBR_SERVOS];
    int pins[NBR_SERVOS];
    int posArray[NBR_SERVOS] = {0}; // angle elevation, angle rotation

    const int MIN_PWM = 16;  // Sur 255
    const int MAX_PWM = 76;  // Sur 255
    const int MIN_ANGLE[NBR_SERVOS] = {0, -90};
    const int MAX_ANGLE[NBR_SERVOS] = {90, 90};
    const int INIT_ANGLE[NBR_SERVOS] = {INIT_ELEVATION, INIT_ROTATION};
    int deg2pwm(int n, int angle);
};

#endif // _SERVOS_H
