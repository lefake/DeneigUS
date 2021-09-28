#ifndef _ENCODER_H
#define _ENCODER_H

#include <SPI.h>
#include "StatusMessage.h"

class Encoder
{
  public:
    Encoder();
    ~Encoder();

    void init(int enc[]);
    int getEncValue(int n);

  private:
    int* encPins;
    uint8_t sendByte(uint8_t message, int pin);
};
#endif // _ENCODER_H
