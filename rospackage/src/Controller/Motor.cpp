#include "Motor.h"
#include "Encoder.h"

Motor::Motor() 
{
  SPIEncoder = Encoder();
}

Motor::~Motor() {}
    
void Motor::init(const int fPin, const int sPin, const int EncoderPin)
{
  forwardPin = fPin;
  speedPin = sPin;
  
  SPIEncoder.init(EncoderPin);
  pinMode(forwardPin, OUTPUT);
  pinMode(speedPin, OUTPUT);
}

void Motor::setPID(float P, float I, float D)
{
  kp = P;
  ki = I;
  kd = D;  
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
  
  v_act = SPIEncoder.getEncVel(dt)*RADIUS;
  if(v_act < -2 || v_act > 2)
    v_act = last_v;

  float e_v = v_des - v_act;
  e_p = e_p + e_v*dt/1000.0;
  if(abs(v_act) < 0.001)
    e_p = 0;
  
  e_a = (last_e_v - e_v)*1000.0/dt;

  last_e_v = e_v;
  last_v = v_act;

  float C = kp*e_v + ki*e_p + kd*e_a;
  if(C < 2.5 && C > -2.5) //dead band
    C = 0;

  setVoltage(C);
} 

void Motor::setVoltage(float volt)
{
  if( volt < 0 )
  {
    digitalWrite(forwardPin, HIGH);
    volt = -volt;
  }
  else 
  {
    digitalWrite(forwardPin, LOW);
  }
  
  if(volt > 24)
    volt = 24;
    
  analogWrite(speedPin, convert(volt));
}

int Motor::convert(float volt)
{
  float absVal = fabs(volt);
  return floatMap(absVal, 0, 24, 0, 255);
}

int Motor::floatMap(float x, float inMin, float inMax, float outMin, float outMax)
{
 return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin; 
}
