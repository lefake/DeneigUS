#ifndef _SONARS_H
#define _SONARS_H

#include <Arduino.h>
#include "Constants.h"
#include "floatarray.pb.h"

#define TIMEOUT_SAFETY_RATIO  1.1

class Sonars
{
  public:
    Sonars();
    ~Sonars();
    
    void init(int trigger[], int echo[]);
    void readPair(int p, FloatArray* msg);

  private:
    int* triggerPins;
    int* echoPins;

    const float soundSpeed = 331.3 + 0.606 * TEMPERATURE;
    const float pusleTimeout = TIMEOUT_SAFETY_RATIO * (((MAX_DIST_DETECTION_M * 2) / soundSpeed) * 1000000);
};

#endif // _SONARS_H
