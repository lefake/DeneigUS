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
const int motorForwardRightPin = 9;
const int motorPwmRightPin = 5;
const int motorForwardLeftPin = 10;
const int motorPwmLeftPin = 6;

// ----------------------- Actuator pins -----------------------
const int actuatorUpPin = 0;
const int actuatorDownPin = 0;
const int actuatorSwitchDownPin = 0;
const int actuatorSwitchUpPin = 0;

#endif // _PINS_H
