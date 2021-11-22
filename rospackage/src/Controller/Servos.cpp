#include "Servos.h"

Servos::Servos() {}
Servos::~Servos() {}
    
void Servos::init(int pins[])
{
  TCCR3B = TCCR3B & B11111000 | B00000100; // for pins 2-3 for PWM frequency of 122.55 Hz
  for(int i = 0; i < NBR_SERVOS; i++){
    servosArray[i].attach(pins[i]);
    //setPos(i, INIT_ANGLE[i]);
  }
}

void Servos::setPos(int n, int pos){
  posArray[n] = deg2pwm(n, pos);
  servosArray[n].write(posArray[n]);
}

int Servos::getPos(int n){
  return posArray[n];
}

int Servos::deg2pwm(int n, int angle){
  //map(value, fromLow, fromHigh, toLow, toHigh)
  return map(angle, MIN_ANGLE[n], MAX_ANGLE[n], MIN_PWM[n], MAX_PWM[n]);
}
