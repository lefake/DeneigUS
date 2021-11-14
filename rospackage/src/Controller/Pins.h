#ifndef _PINS_H
#define _PINS_H

#include "Constants.h"

// ----------------------- Sonars pins -----------------------
const int sonarsTriggerPin[NBS_SONARS] { 22, 24, 26, 28, 30, 32, 34, 36, 38, 40, 42, 44, 46, 48 };    // TODO : Make sure pins are in pairs
const int sonarsEchoPins[NBS_SONARS] { 23, 25, 27, 29, 31, 33, 35, 37, 39, 41, 43, 45, 47, 49 };

// ----------------------- Encoder pins ----------------------
const int mosiPin = 51;
const int misoPin = 50;
const int clkPin = 52;
const int encoderCSPins[NBS_ENCODERS] { 53 };

// ----------------------- Servos pins ----------------------
const int servoPins[NBR_SERVOS] { 2, 3 }; // Elevation, rotation

// ----------------------- Motors pins -----------------------
const int motorForwardRight = 5;
const int motorPwmRight = 4;
const int motorForwardLeft = 8;
const int motorPwmLeft = 7;

#endif // _PINS_H
