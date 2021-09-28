#ifndef _PINS_H
#define _PINS_H

#include "Constants.h"

// ----------------------- Encoder pins ----------------------
const int mosiPin = 51;
const int misoPin = 50;
const int clkPin = 52;
const int encoderCSPins[NBS_ENCODERS] { 53 };

// ----------------------- Sonars pins -----------------------
const int sonarsTriggerPin[NBS_SONARS] { 23, 25 };
const int sonarsEchoPins[NBS_SONARS] { 22, 24 };

// ----------------------- Motors pins -----------------------
const int motorForwardRight = 5;
const int motorBackwardRight = 6;
const int motorPwmRight = 4;
const int motorForwardLeft = 8;
const int motorBackwardLeft = 9;
const int motorPwmLeft = 7;

#endif // _PINS_H
