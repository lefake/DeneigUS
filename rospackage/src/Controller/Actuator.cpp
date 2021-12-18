#include "Actuator.h"

Actuator::Actuator(ErrorHandler* e) 
{
  errorHandler = e;
}

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

  readPosition();
}

void Actuator::setDir(int dir)
{
  if (dir == currentPos)
    currentCmd = UNKNOWN;
  else
    currentCmd = dir;
  
  if(currentCmd == UP)
    setUp();
  else if(currentCmd == DOWN)
    setDown(); 
  else
    disable();
}

int Actuator::getCurrentPos()
{
  readPosition();
  return currentPos;
}

void Actuator::disable()
{
  digitalWrite(relayUpPin, LOW);
  digitalWrite(relayDownPin, LOW); 
}

void Actuator::setDown()
{
  disable();
  digitalWrite(relayDownPin, HIGH);
}

void Actuator::setUp()
{
  disable();
  digitalWrite(relayUpPin, HIGH);
}

void Actuator::readPosition()
{
  if(digitalRead(downSwitchPin) == LOW && digitalRead(upSwitchPin) == LOW)
  {
    currentPos = UNKNOWN;
  } 
  else if(digitalRead(downSwitchPin) == HIGH && digitalRead(upSwitchPin) == LOW)
  {
    if (currentCmd != UP)
      disable();
    currentPos = DOWN;
  }
  else if(digitalRead(downSwitchPin) == LOW && digitalRead(upSwitchPin) == HIGH)
  {
    if (currentCmd != DOWN)
      disable();
    currentPos = UP;
  }  
  else if(digitalRead(downSwitchPin) == HIGH && digitalRead(upSwitchPin) == HIGH)
  {
    currentPos = UNKNOWN;
    disable();
    errorHandler->sendStatus(FATAL, ACTUATOR_DEVICE, "Both switch are pressed at the same time");
  }
}
