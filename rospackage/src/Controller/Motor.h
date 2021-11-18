#ifndef _MOTOR_H
#define _MOTOR_H

#include <Arduino.h>
#include "Constants.h"
#include "Encoder.h"

class Motor {
  public:
    Motor();
    ~Motor();

    void init(const int, const int, const int EncoderPin);
    float getSpeed();
    void setPID(float P, float I, float D);
    void commandSpeed(float command);
    void computePID(); 
    void setVoltage(float);

  private:
    int forwardPin;
    int speedPin;
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
    
    Encoder SPIEncoder;

    void updatedt();
    int floatMap(float, float, float, float, float);
    int convert(float);
    

    
};

#endif // _MOTOR_H
