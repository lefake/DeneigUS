#include "Acknowledge.h"

AckHandler::AckHandler() { }

bool AckHandler::acknowldgeArduino(char* msg)
{
  if (String(msg).toInt() == ACK_REQUEST_ID)
  {
    Serial.print("{");
    Serial.print(IdName[ARDUINO_ID]);
    Serial.print("}");
    return true;
  }
  else
    return false;
}
