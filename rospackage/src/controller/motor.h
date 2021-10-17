#ifndef MOTOR_H
#define MOTOR_H
#include "Arduino.h"
#include "constants.h"

#define CALC(l, r) (l + ((r * TRACK_DIST) / 2))

class Motor {
  public:
    Motor();
    ~Motor();

    void init(const int forward_pin, const int backward_pin, const int speed_pin);
    void set_speed(float);

  private:
    int _forward_pin;
    int _backward_pin;
    int _speed_pin;

    int float_map(float x, float in_min, float in_max, float out_min, float out_max);
    int convert(float percentage);
};

#endif
