#include "Motor.h"
#include "Encoder.h"

Motor::Motor(ErrorHandler* e) 
{
  SPIEncoder = new Encoder(e);
  errorHandler = e;
}

Motor::~Motor() {}
    
void Motor::init(const int bPin, const int sPin, const int encoderPin, const int latch, bool reverse)
{
  backwardPin = bPin;
  speedPin = sPin;
  latchPin = latch;
  
  SPIEncoder->init(encoderPin, reverse);
  pinMode(backwardPin, OUTPUT);
  pinMode(speedPin, OUTPUT);
  pinMode(latchPin, INPUT);
}

void Motor::setPID(float P, float I, float D)
{
  kp = P;
  ki = I;
  kd = D;

  reset();
}

float Motor::getSpeed()
{
  return v_act;
}

void Motor::updatedt()
{
  dt = millis() - lastMillis;
  lastMillis = millis();
}


void Motor::commandSpeed(float command)
{
    v_des = command;
}

void Motor::computePID()
{
  updatedt();
  
  v_act = SPIEncoder->getEncVel(dt)*RADIUS;
  
  if(v_act < -2 || v_act > 2)
    v_act = last_v;

  float e_v = v_des - v_act;

  // Saturation
  if(abs(e_p) <= 24)
    e_p += e_v*dt/1000.0;

  if(abs(v_act) < 0.001)
    e_p = 0;

  if(abs(v_des) < 0.01)
    e_p = 0;
  
  e_a = (last_e_v - e_v)*1000.0/dt;

  float C = kp*e_v + ki*e_p + kd*e_a; 
  setVoltage(C);

  last_e_v = e_v;
  last_v = v_act;
} 

void Motor::setVoltage(float volt)
{
  current_c = volt;
  // Test sign
  if ((last_c * volt) < 0)
  {
    analogWrite(speedPin, 0);
    if (abs(v_act) < 0.01)
    {
      sens = volt < 0;
      digitalWrite(backwardPin, sens);     // Set to HIGH if backward
      if (digitalRead(latchPin) == sens)
      {
        last_c = volt;
      }
    }
  }
  else
  {
    analogWrite(speedPin, (int) (abs(volt)*10.625));
  }
}

float Motor::getCurrentCmd()
{
  return v_des;
}

float Motor::getCurrentOutput()
{
  return current_c;
}

float Motor::getDir()
{
  return sens;
}

void Motor::disable()
{
  setVoltage(0);
  commandSpeed(0);
  reset();
}

void Motor::reset()
{
  v_act = 0;
  v_des = 0;
  e_p = 0;
  e_a = 0;
  last_v = 0;
  last_e_v = 0;
  sens = 0;
  last_c = 1;   // Init as positive value
  lastLatch = 0;
}
