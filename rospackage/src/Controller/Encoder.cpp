#include "Encoder.h"

Encoder::Encoder(){ }

Encoder::~Encoder(){ }

void Encoder::init(int enc[])
{
  encPins = enc;

  pinMode(clkPin, OUTPUT);
  pinMode(mosiPin, OUTPUT);
  pinMode(misoPin, INPUT);

  for (int i = 0; i < NBS_ENCODERS; i++)
  {
    pinMode(encPins[i], OUTPUT);
    digitalWrite(encPins[i], HIGH);
  }

  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  SPI.begin();

  for (int i = 0; i < NBS_ENCODERS; i++)
  {
    lastRead[i] = getEncValue(i);
  }
}

double Encoder::getEncVel(int n, long dt)
{
  double maxTickCount = MAX_TICK_SAMPLE_SAFE(dt);
  int current = getEncValue(n);
  double diff = current - lastRead[n];
  
  // A turn
  if (abs(diff)/maxTickCount > 1)
  {
    if (diff > 0)
      diff = -lastRead[n]-(NBS_TICK_PER_REV-1.0-current);
    else
      diff = (NBS_TICK_PER_REV-1.0-lastRead[n])+current;
  }

  lastRead[n] = current;
  
  double deg = diff * 360.0 / NBS_TICK_PER_REV;
  return deg * 1000.0 / dt;
}

int Encoder::getEncValue(int n)
{
  int encCurrPin = encPins[n];
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
  else
  {
    sendStatusWithMessage(ERROR, ENCODER_DEVICE, "Encoder #" + String(n) + " bad read (" + receivedMessage + ")");
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
