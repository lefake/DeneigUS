#ifndef SONARS_H
#define SONARS_H

#include <HCSR04.h>
#include <std_msgs/Float32MultiArray.h>
#include "constants.h"

class Sonars
{
  public:
    Sonars(int trigger_pin, int echo_pins[NBS_SONARS]);
    float* getDistancesCm();
    void getDistancesRos( std_msgs::Float32MultiArray* msg );

  private:
    HCSR04* hc;
    int _trigger_pin;
    int _echo_pins[NBS_SONARS]; 
};

#endif // SONARS_H
