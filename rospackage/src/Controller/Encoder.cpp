#include "Encoder.h"

Encoder::Encoder(){ }

Encoder::~Encoder(){ }

void Encoder::init(int enc)
{
  encPin = enc;

  pinMode(clkPin, OUTPUT);
  pinMode(mosiPin, OUTPUT);
  pinMode(misoPin, INPUT);

  pinMode(encPin, OUTPUT);
  digitalWrite(encPin, HIGH);
    
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  SPI.begin();

  delay(75);

  lastRead = getEncValue();
 
}

float Encoder::getEncVel(long dt)
{
  float maxTickCount = MAX_TICK_SAMPLE_SAFE(dt);
  int current = getEncValue();
  float diff = current - lastRead;
  
  // A turn
  if (abs(diff)/maxTickCount > 1)
  {
    if (diff > 0)
      diff = -lastRead-(NBS_TICK_PER_REV-1.0-current);
    else
      diff = (NBS_TICK_PER_REV-1.0-lastRead)+current;
  }

  lastRead = current;
  
  float deg = diff * 2 * M_PI / NBS_TICK_PER_REV;
  return deg * 1000.0 / dt;
}

int Encoder::getEncValue()
{
  int encCurrPin = encPin;
  uint8_t receivedMessage = WAIT;
  uint16_t pos = -1;
  
  sendByte(RD_POS, encCurrPin);
  
  while(receivedMessage == WAIT)
  {
    receivedMessage = sendByte(NOP_A5, encCurrPin);
  }
  
  if(receivedMessage == RD_POS)
  {
    uint8_t msbPosition = sendByte(NOP_A5, encCurrPin);
    uint8_t lsbPosition = sendByte(NOP_A5, encCurrPin);
    pos = msbPosition << 8 | lsbPosition;
  }
  else if (pos > NBS_TICK_PER_REV || pos < 0)
  {
    sendStatusWithMessage(ERROR, ENCODER_DEVICE, "Encoder #" + String(encPin) + " bad read (" + receivedMessage + ")");
  }
  else
  {
    sendStatusWithMessage(ERROR, ENCODER_DEVICE, "Encoder #" + String(encPin) + " bad read (" + receivedMessage + ")");
  }

  return pos;
}

uint8_t Encoder::sendByte(uint8_t message, int pin)
{
   digitalWrite(pin, LOW);
   delayMicroseconds(3);
   uint8_t response = SPI.transfer(message);
   digitalWrite(pin, HIGH);
   delayMicroseconds(20);
   return response;
}
