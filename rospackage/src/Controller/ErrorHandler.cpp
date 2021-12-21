#include "ErrorHandler.h"

ErrorHandler::ErrorHandler() { }

ErrorHandler::~ErrorHandler() { }

void ErrorHandler::initSafety(const int eStopP, const int eStopStateP, const int controllerEStopP, const int sensorEStopP)
{
  setCommonPins(eStopP, eStopStateP, controllerEStopP, sensorEStopP);
  pinMode(controllerEStopPin, INPUT);
  pinMode(sensorEStopPin, INPUT);
}

void ErrorHandler::initController(const int eStopP, const int eStopStateP, const int controllerEStopP, const int sensorEStopP)
{
  setCommonPins(eStopP, eStopStateP, controllerEStopP, sensorEStopP);
  digitalWrite(controllerEStopPin, LOW);
  pinMode(controllerEStopPin, OUTPUT);
  pinMode(sensorEStopPin, INPUT);
}

void ErrorHandler::initSensors(const int eStopP, const int eStopStateP, const int controllerEStopP, const int sensorEStopP)
{
  setCommonPins(eStopP, eStopStateP, controllerEStopP, sensorEStopP);
  pinMode(controllerEStopPin, INPUT);
  digitalWrite(sensorEStopPin, LOW);
  pinMode(sensorEStopPin, OUTPUT);
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

bool ErrorHandler::getEStopState()
{
  return currentEStopState;
}

void ErrorHandler::setEStop(bool state)
{
  digitalWrite(eStopPin, state);
}

void ErrorHandler::readForwardedEStop()
{
  digitalWrite(eStopPin, digitalRead(controllerEStopPin) || digitalRead(sensorEStopPin));
}

void ErrorHandler::readDebouncedEStop()
{
  bool readState = digitalRead(eStopStatePin);
  
  if (lastEStopState != readState)
    lastDebounceTime = millis();

  if(readState != currentEStopState && (millis() - lastDebounceTime) > delayDebounceInterval)
  {
    currentEStopState = readState;

    if (currentEStopState) // EStop disable
      digitalWrite(eStopPin, false);
  }

  lastEStopState = readState;
}

void ErrorHandler::setCommonPins(const int eStopP, const int eStopStateP, const int controllerEStopP, const int sensorEStopP)
{
  eStopPin = eStopP;
  eStopStatePin = eStopStateP;
  controllerEStopPin = controllerEStopP;
  sensorEStopPin = sensorEStopP;

  digitalWrite(eStopPin, LOW);
  pinMode(eStopPin, OUTPUT);
  pinMode(eStopStatePin, INPUT);
}
