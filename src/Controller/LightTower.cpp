#include "LightTower.h"

LightTower::LightTower() {}

LightTower::~LightTower() {}

void LightTower::init(const int pins[3])
{
  lightPins = pins;

  for (int i = 0; i < 3; ++i)
    pinMode(lightPins[i], OUTPUT);
}

void LightTower::toggle(const int pin, const int state)
{
  digitalWrite(lightPins[pin], state);
}
