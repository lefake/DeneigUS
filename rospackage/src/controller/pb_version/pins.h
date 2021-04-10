#ifndef PINS_H
#define PINS_H

#include "constants.h"

// ----------------------- Sonars pins -----------------------
const int sonars_trigger_pin[NBS_SONARS] { 2, 3 };
const int sonars_echo_pins[NBS_SONARS] { 4, 5 };

// ----------------------- Motors pins -----------------------
const int forw_left = 3;
const int back_left = 4;
const int pwm_left = 2;
const int forw_right = 6;
const int back_right = 7;
const int pwm_right = 5;

#endif // PINS_H
