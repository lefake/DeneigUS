#include "ErrorHandler.h"

ErrorHandler::ErrorHandler() { }

ErrorHandler::~ErrorHandler() { }

void ErrorHandler::init()
{
  pinMode(eStopStatePin, INPUT);
  pinMode(eStopPin, OUTPUT);
  digitalWrite(eStopPin, LOW);
}

void ErrorHandler::sendStatus(int level, int type, String msg)
{
  if (level <= ERROR)
    setEStop(true);
  
  if (level <= LOG_LEVEL)
  {
    Serial.print("{");
    Serial.print(level);
    Serial.print(";");
    Serial.print(type);
    Serial.print(";");
    Serial.print(msg);
    Serial.print("}");
  }
  
}

void ErrorHandler::sendStatus(int level, int type)
{
  sendStatus(level, type, "");
}

void ErrorHandler::sendNotInit(int type)
{
  sendStatus(FATAL, type, "Init failed");
}

void ErrorHandler::setEStop(bool state)
{
  digitalWrite(eStopPin, state);
}

bool ErrorHandler::readEStop()
{
  return digitalRead(eStopStatePin);
}

bool ErrorHandler::getEStop()
{
  return eStopState;
}

int ErrorHandler::readDebouncedEStop()
{
  bool readState = digitalRead(eStopStatePin);
  bool changed = false;
  
  if (lastEStopState != readState)
    lastDebounceTime = millis();

  if(readState != eStopState && (millis() - lastDebounceTime) > delayDebounceInterval)
  {
    eStopState = readState;
    changed = true;
  }

  lastEStopState = readState;

  if (changed)
    return eStopState;
  else
    return -1;
}
