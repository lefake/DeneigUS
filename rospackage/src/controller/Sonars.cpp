#include "Sonars.h"

Sonars::Sonars() { _echo_pins = (int *) malloc(sizeof(int)*NBS_SONARS); }

Sonars::~Sonars() { delete[] _echo_pins; }

void Sonars::init(int trigger, int echo[])
{
  _trigger_pin = trigger;
  _echo_pins = echo;

  pinMode(_trigger_pin, OUTPUT);

  for (int i = 0; i < NBS_SONARS; i++)
    pinMode(_echo_pins[i], INPUT);
}

float Sonars::dist(int n)
{
  digitalWrite(_trigger_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(_trigger_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(_trigger_pin, LOW);
  
  noInterrupts();
  float d = pulseIn(_echo_pins[n], HIGH);
  interrupts();

  float speedOfSoundInCmPerMs = 0.03313 + 0.0000606 * TEMPERATURE; // Cair ≈ (331.3 + 0.606 ⋅ ϑ) m/s
  float distanceCm = d / 2.0 * speedOfSoundInCmPerMs;
  
  if (distanceCm <= MIN_DIST_DETECTION_CM) 
    return -1.0;
  else if (distanceCm > MAX_DIST_DETECTION_CM)
    return -2.0 ;
  else
    return distanceCm;
}

void Sonars::getDistancesRos( std_msgs::Float32MultiArray* msg )
{
  float distances[NBS_SONARS] = { 0 };

  for (int i = 0; i < NBS_SONARS; ++i)
    distances[i] = dist(i);

  for (int i = 0; i < NBS_SONARS + 1; ++i)
  {
    msg->data[i * NBS_DATA_SONRARS] = (distances[i] > DIST_THRESHOLD_CM) ? 0 : 1;
    msg->data[i * NBS_DATA_SONRARS + 1] = distances[i];
  }
}
