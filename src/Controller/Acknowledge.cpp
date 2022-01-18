#include "Acknowledge.h"

AckHandler::AckHandler() : id(-1) { }

bool AckHandler::acknowldgeArduino(char* msg)
{
  if (String(msg).toInt() == ACK_REQUEST_ID)
  {
    Serial.print("{");
    Serial.print(IdName[id]);
    Serial.print("}");
    acked = true;
    return true;
  }
  else
    return false;
}

bool AckHandler::getAcked()
{
  return acked;
}

void AckHandler::writeIdToEEPROM(int idToWrite)
{
  EEPROM.update(ID_EEPROM_ADDRESS, idToWrite);
  id = idToWrite;
}

void AckHandler::readIdFromEEPROM()
{
  id = EEPROM.read(ID_EEPROM_ADDRESS);
}

int AckHandler::getId()
{
  return id;
}
