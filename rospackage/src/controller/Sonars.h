#ifndef SONARS_H
#define SONARS_H

#include <std_msgs/Float32MultiArray.h>
#include "Arduino.h"
#include "constants.h"

class Sonars
{
  public:
    Sonars();
    ~Sonars();
    
    void init(int trigger, int echo[]);
    void getDistancesRos( std_msgs::Float32MultiArray* msg );

  private:
    float dist(int n);
  
    int _trigger_pin;
    int* _echo_pins; 
};

#endif // SONARS_H
