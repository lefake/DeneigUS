#ifndef _ACKNOWLEDGE_H
#define _ACKNOWLEDGE_H

#include <Arduino.h>
#include <EEPROM.h>
#include "Constants.h"

#define ACK_REQUEST_ID    42 

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
    int getId();
    bool getAcked();
    
    void writeIdToEEPROM(int id);
    void readIdFromEEPROM();
  
  private:
    int id;
    bool acked = false;
    String IdName[_NBS_ID] = { "CONTROLLER", "SENSORS", "SAFETY" };
};

#endif // _ACKNOWLEDGE_H
