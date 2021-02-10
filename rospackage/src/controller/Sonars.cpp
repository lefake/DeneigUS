#include "Sonars.h"

Sonars::Sonars(int trigger_pin, int echo_pins[NBS_SONARS])
{
  _trigger_pin = trigger_pin;
  memcpy(_echo_pins, echo_pins, sizeof(echo_pins[0])*NBS_SONARS);

  hc = new HCSR04(_trigger_pin, _echo_pins, NBS_SONARS);
}

float* Sonars::getDistancesCm()
{
  float* distances = new float[NBS_SONARS] { 0 };

  for (int i = 0; i < NBS_SONARS; ++i)
    distances[i] = hc->dist(i);
    
  return distances;
}

void Sonars::getDistancesRos( std_msgs::Float32MultiArray* msg )
{
  float* distances = getDistancesCm();

  for (int i = 0; i < NBS_SONARS; ++i)
  {
    msg->data[i*NBS_DATA_SONRARS] = (distances[i] > DIST_THRESHOLD_CM) ? 0 : 1;
    msg->data[i*NBS_DATA_SONRARS+1] = distances[i];
  }
}
