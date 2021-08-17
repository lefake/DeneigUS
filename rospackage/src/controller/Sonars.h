#ifndef SONARS_H
#define SONARS_H

#include "Arduino.h"
#include "constants.h"

class Sonars
{
  public:
    Sonars();
    ~Sonars();
    
    void init(int trigger[], int echo[]);
    float dist(int n);

  private:
    int* _trigger_pins;
    int* _echo_pins;

    const float soundSpeed = 331.3 + 0.606 * TEMPERATURE;
    const float pusleTimeout = 1.1 * (((MAX_DIST_DETECTION_M * 2) / soundSpeed) * 1000000);
    
};

#endif // SONARS_H
