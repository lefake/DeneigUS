#ifndef _ACKNOWLEDGE_H
#define _ACKNOWLEDGE_H

#include <Arduino.h>
#include <EEPROM.h>
#include "Constants.h"

#define ACK_REQUEST_ID    42
#define ARDUINO_ID        CONTROLLER

enum ID {
  CONTROLLER = 0,
  SENSORS,
  SAFETY,

  _NBS_ID
};

class AckHandler
{
  public:
    AckHandler();
    bool acknowldgeArduino(char* msg);
  
  private:
    String IdName[_NBS_ID] = { "CONTROLLER", "SENSORS", "BATTERY" };
};

#endif // _ACKNOWLEDGE_H
