#ifndef PINS_H
#define PINS_H

#include "constants.h"

// ----------------------- Sonars pins -----------------------
const int sonars_trigger_pin[NBS_SONARS] { 23, 25 };
const int sonars_echo_pins[NBS_SONARS] { 22, 24 };

// ----------------------- Motors pins -----------------------
const int forw_right = 5;
const int back_right = 6;
const int pwm_right = 4;
const int forw_left = 8;
const int back_left = 9;
const int pwm_left = 7;

#endif // PINS_H
