#include "LightTower.h"

LightTower::LightTower() {}

LightTower::~LightTower() {}

void LightTower::init(int pins[])
{
  lightPins = pins;
  pinMode(lightPins[RED], OUTPUT);
  pinMode(lightPins[YELLOW], OUTPUT);
  pinMode(lightPins[GREEN], OUTPUT);
}

void LightTower::toggle(FloatArray cmd)
{
  digitalWrite(lightPins[(int) cmd.data[0]], cmd.data[1]);
}
