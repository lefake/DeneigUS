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

double Motor::getSpeed()
{
  updatedt();
  return SPIEncoder.getEncVel(dt);
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
  
  v_act = getSpeed()*RADIUS;  
  if(v_act < -2 || v_act > 2)
    v_act = last_v;

  double e_v = v_des - v_act;
  //if(e_v < 0.05 && e_v > -0.05) //dead band
  //  e_v =0;
  
  e_p = e_p + e_v*dt/1000.0;
  if(v_act == 0)
    e_p = 0;
  
  e_a = (last_e_v - e_v)*1000.0/dt;

  last_e_v = e_v;
  last_v = v_act;

  float C = kp*e_v + ki*e_p + kd*e_a;

  setVoltage(C);
} 

void Motor::setVoltage(float volt)
{

  if( volt < 0 ){
      digitalWrite(forwardPin, HIGH);
      volt = -volt;
  }
  if( volt > 0 ){
      digitalWrite(forwardPin, LOW);
  }
  if( volt > 24)
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
