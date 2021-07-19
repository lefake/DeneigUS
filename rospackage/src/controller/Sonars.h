#ifndef SONARS_H
#define SONARS_H

#include "Arduino.h"
#include "constants.h"

class Sonars
{
  public:
    Sonars();
    ~Sonars();
    
    void init(int trigger[], int echo[] );
    float dist(int n);

  private:
    int* _trigger_pins;
    int* _echo_pins;
};

#endif // SONARS_H
