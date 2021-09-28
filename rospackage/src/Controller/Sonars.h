#ifndef _SONARS_H
#define _SONARS_H

#include "Arduino.h"
#include "Constants.h"

class Sonars
{
  public:
    Sonars();
    ~Sonars();
    
    void init(int trigger[], int echo[]);
    float dist(int n);

  private:
    int* triggerPins;     // TODO : Why dynamically assigned?
    int* echoPins;

    const float soundSpeed = 331.3 + 0.606 * TEMPERATURE;
    const float pusleTimeout = 1.1 * (((MAX_DIST_DETECTION_M * 2) / soundSpeed) * 1000000);
};

#endif // _SONARS_H
