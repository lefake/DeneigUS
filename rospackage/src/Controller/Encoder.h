#ifndef _ENCODER_H
#define _ENCODER_H

#include <SPI.h>
#include "StatusMessage.h"
#include "Pins.h"
#include "Constants.h"

#define MAX_SPEED_TICK_SEC          MAX_SPEED_RPM/60.0*NBS_TICK_PER_REV
#define MAX_TICK_SAMPLE_SAFE(dt)    MAX_SPEED_TICK_SEC/(1000.0/dt)*TICK_SAFTY_RATIO

class Encoder
{
  public:
    Encoder();
    ~Encoder();

    void init(int enc[]);
    int getEncValue(int n);
    double getEncVel(int n, long dt);

  private:
    int* encPins;
    uint8_t sendByte(uint8_t message, int pin);
    int lastRead[NBS_ENCODERS] = { 0 };
    
};
#endif // _ENCODER_H
