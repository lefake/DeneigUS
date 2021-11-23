#ifndef _LIGHTTOWER_H
#define _LIGHTTOWER_H

#include <Arduino.h>
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

    void init(const int [3]);
    void toggle(const int , const int );

  private:
    const int* lightPins;
};

#endif // _LIGHTTOWER_H
