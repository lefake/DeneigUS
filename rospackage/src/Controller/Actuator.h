#ifndef _ACTUATOR_H
#define _ACTUATOR_H

#include <Arduino.h>
#include "ErrorHandler.h"
#include "Constants.h"

class Actuator{
  public:
    Actuator(ErrorHandler* e);
    ~Actuator();

    void init(const int, const int, const int, const int);
    void setDir(int);
    int getCurrentPos();

  private:
    ErrorHandler* errorHandler;
  
    int downSwitchPin;
    int upSwitchPin;
    int relayUpPin;
    int relayDownPin;

    int currentCmd;
    int currentPos;

    void disable();
    void setDown();
    void setUp();
    void readPosition();

    enum Position
    {
      DOWN = -1,
      UNKNOWN = 0,
      UP = 1,
    };
};

#endif // _MOTOR_H
