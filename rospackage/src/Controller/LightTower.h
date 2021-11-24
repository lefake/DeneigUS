#ifndef _LIGHTTOWER_H
#define _LIGHTTOWER_H

#include <Arduino.h>
#include "floatarray.pb.h"
#include "Constants.h"

enum COLORS {
  RED,
  YELLOW,
  GREEN
};  

class LightTower {
  public:
    LightTower();
    ~LightTower();

    void init(int pins[]);
    void toggle(FloatArray);

  private:
    int* lightPins;
};

#endif // _LIGHTTOWER_H
