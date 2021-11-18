#include "Actuator.h"

Actuator::Actuator() {}
Actuator::~Actuator() {}
    
void Actuator::init(const int upPin, const int downPin, const int rUpPin, const int rDownPin)
{
  upSwitchPin = upPin;
  downSwitchPin = downPin;

  relayUpPin = rUpPin;
  relayDownPin = rDownPin;

  pinMode(upSwitchPin, INPUT);
  pinMode(downSwitchPin, INPUT);

  pinMode(relayUpPin, OUTPUT);
  pinMode(relayDownPin, OUTPUT);
}

void Actuator::setDir(int dir)
{
  cmd = dir;
  if(dir == UP)
  {
    digitalWrite(relayUpPin, HIGH);
    digitalWrite(relayDownPin, LOW);
  }
  else if(dir == DOWN)
  {
    digitalWrite(relayUpPin, LOW);
    digitalWrite(relayDownPin, HIGH); 
  }
  else if (dir == UNKOWN)
  {
    disable();     
  }
  else
  {
    sendStatusWithMessage(ERROR, ACTUATOR_DEVICE, "No a valid command");
  }
}

int Actuator::getPos()
{
  if( digitalRead(downSwitchPin) == LOW && digitalRead(upSwitchPin) == LOW)
  {
    return UNKOWN;
  } 
  else if( digitalRead(downSwitchPin) == HIGH && digitalRead(upSwitchPin) == LOW)
  {
    if (cmd != UP)
      disable();
    return DOWN;
  }
  else if( digitalRead(downSwitchPin) == LOW && digitalRead(upSwitchPin) == HIGH)
  {
    if (cmd != DOWN)
      disable();
    return UP;
  }  
  else if( digitalRead(downSwitchPin) == HIGH && digitalRead(upSwitchPin) == HIGH)
  {
    disable();
    sendStatusWithMessage(FATAL, ACTUATOR_DEVICE, "Both switch are pressed at the same time");
  }
}

void Actuator::disable()
{
  digitalWrite(relayUpPin, LOW);
  digitalWrite(relayDownPin, LOW); 
}
