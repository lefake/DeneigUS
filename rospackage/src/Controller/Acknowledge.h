#ifndef _ACKNOWLEDGE_H
#define _ACKNOWLEDGE_H

#include <Arduino.h>
#include "Constants.h"

#define ACK_REQUEST_ID    42
#define ARDUINO_ID        CONTROLLER

class AckHandler
{
  public:
    AckHandler();
    bool acknowldgeArduino(char* msg);
  
  private:
    String IdName[3] = { "CONTROLLER", "SENSORS", "SAFETY" };
};

#endif // _ACKNOWLEDGE_H
