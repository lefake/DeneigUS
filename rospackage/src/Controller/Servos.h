#ifndef _SERVOS_H
#define _SERVOS_H

#include <Arduino.h>
#include "Constants.h"
#include "StatusMessage.h"
#include <Servo.h>

enum SERVOS 
{
  ROTATION = 0,
  ELEVATION
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

    const int MIN_PWM[NBR_SERVOS] = {16, 255};
    const int MAX_PWM[NBR_SERVOS] = {76, 0};
    const int MIN_ANGLE[NBR_SERVOS] = {-90, 0};
    const int MAX_ANGLE[NBR_SERVOS] = {90, 90};
    const int INIT_ANGLE[NBR_SERVOS] = {INIT_ELEVATION, INIT_ROTATION};
    int deg2pwm(int n, int angle);
};

#endif // _SERVOS_H
