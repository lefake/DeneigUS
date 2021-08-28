#include "StatusMessage.h"

void sendStatus(int level, int type)
{
  sendStatusWithMessage(level, type, "");
}

void sendStatusNotInitialize(int level, int type)
{
  sendStatusWithMessage(level, type, "Init failed");
}

void sendStatusWithMessage(int level, int type, String msg)
{
  Serial.print("{");
  if (level != -1)
  {
    Serial.print(level);
    Serial.print(";");
  }
  Serial.print(type);
  Serial.print(";");
  Serial.print(msg);
  Serial.println("}");
}
