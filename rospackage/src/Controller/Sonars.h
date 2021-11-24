#ifndef _SONARS_H
#define _SONARS_H

#include <Arduino.h>
#include "Constants.h"
#include "floatarray.pb.h"
#include "ErrorHandler.h"

#define TIMEOUT_SAFETY_RATIO  1.25

class Sonars
{
  public:
    Sonars(ErrorHandler* e);
    ~Sonars();
    
    void init(int trigger[], int echo[]);
    void readPair(int p, FloatArray* msg);

  private:
    ErrorHandler* errorHandler;
  
    int* triggerPins;
    int* echoPins;

    const float soundSpeed = 331.3 + 0.606 * TEMPERATURE;
    const unsigned long pusleTimeout = TIMEOUT_SAFETY_RATIO * (((MAX_DIST_DETECTION_M * 2.0) / soundSpeed) * 1000000.0);
};

#endif // _SONARS_H
