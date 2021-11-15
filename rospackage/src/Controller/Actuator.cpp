#include "Actuator.h"

Actuator::Actuator() {}
Actuator::~Actuator() {}
    
void Actuator::init(const int fPin, const int bPin, const int r1Pin, const int r2Pin)
{
  forwardLSPin = fPin;
  backwardLSPin = bPin;

  relay1Pin = r1Pin;
  relay2Pin = r2Pin;

  pinMode(forwardLSPin, INPUT);
  pinMode(backwardLSPin, INPUT);

  pinMode(relay1Pin, OUTPUT);
  pinMode(relay2Pin, OUTPUT);
}

void Actuator::enable(int dir)
{
  if(dir == 1)
  {
    digitalWrite(relay1Pin, HIGH);
    digitalWrite(relay2Pin, LOW);
  }
  else if( dir == -1)
  {
    digitalWrite(relay1Pin, LOW);
    digitalWrite(relay2Pin, HIGH); 
  }
  else
  {
    disable();    
//    error msg wrong dir 
  }
}

int Actuator::getPos()
{
  if( digitalRead(forwardLSPin) == LOW && digitalRead(backwardLSPin) == LOW)
  {
    return 0;
  } 
  else if( digitalRead(forwardLSPin) == HIGH && digitalRead(backwardLSPin) == LOW)
  {
    disable();
    return 1;
  }
  else if( digitalRead(forwardLSPin) == LOW && digitalRead(backwardLSPin) == HIGH)
  {
    disable();
    return -1;
  }  
  else if( digitalRead(forwardLSPin) == HIGH && digitalRead(backwardLSPin) == HIGH)
  {
//    error msg wrong pos
    disable();
  }
}

void Actuator::disable()
{
  digitalWrite(relay1Pin, LOW);
  digitalWrite(relay2Pin, LOW); 
}
