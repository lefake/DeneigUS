#ifndef _MOTOR_H
#define _MOTOR_H

#include <Arduino.h>
#include "Constants.h"
#include "StatusMessage.h"
#include "Encoder.h"

class Motor {
  public:
    Motor();
    ~Motor();

    void init(const int, const int, const int, const int, bool);
    void setPID(float P, float I, float D);
    void commandSpeed(float command);
    void computePID(); 
    void setVoltage(float);
    void disable();

    float getSpeed();
    float getCurrentCmd();
    float getCurrentOutput();
    float getDir();

  private:
    int backwardPin;
    int speedPin;
    int latchPin;
    float kp = 13.0;
    float ki = 11.0;
    float kd = 0.7;

    long dt = 0;
    long lastMillis = 0;

    float v_act = 0;
    float v_des = 0;
    float e_p = 0;
    float e_a = 0;
    float last_v = 0;
    float last_e_v = 0;

    float sens = 0;
    float last_c = 1;   // Init as positive value
    float current_c;

    int lastLatch = 0;
    
    Encoder SPIEncoder;

    void updatedt();
    void reset();
};

#endif // _MOTOR_H
