#ifndef _PINS_H
#define _PINS_H

#include "Constants.h"

// ----------------------- Sonars pins -----------------------
const int sonarsTriggerPin[NBS_SONARS] { 22, 24, 38, 42, 48, 46, 36, 30, A6, 40, 26, 32, 34, 28 };
const int sonarsEchoPins[NBS_SONARS] { 23, 25, 39, 43, 49, 47, 37, 31, A7, 41, 27, 33, 35, 29 };
// Sonar 8 -> J25

// ----------------------- Encoder pins ----------------------
const int mosiPin = 51;
const int misoPin = 50;
const int clkPin = 52;
const int csEncoderL = 29;
const int csEncoderR = 25;

// ----------------------- Servos pins ----------------------
const int servoPins[NBR_SERVOS] { 2, 3 }; // Elevation, rotation

// ----------------------- Motors pins -----------------------
const int motorForwardRight = 9;
const int motorPwmRight = 5;
const int motorForwardLeft = 10;
const int motorPwmLeft = 6;

#endif // _PINS_H
