#ifndef _STATUSMESSAGE_H
#define _STATUSMESSAGE_H

#include <Arduino.h>

void sendStatus(int level, int type);
void sendStatusWithMessage(int level, int type, String msg);
void sendStatusNotInitialized(int type);

enum STATUS_LVL 
{
  FATAL = 0,
  ERROR,
  WARNING,
  INFO,
  DEBUG,

  _NBS_LVL
};

enum STATUS_TYPE 
{
  SERIAL_COMMUNICATION = 0,
  ENCODING_PB,
  DECODING_PB,
  TOPICS,
  GPS_DEVICE,
  IMU_DEVICE,
  SONARS_DEVICE,
  MOTOR_BLOW_DEVICE,
  MOTOR_PROP_DEVICE,
  SERVOS_DEVICE,
  ENCODER_DEVICE,
  ACTUATOR_DEVICE,
  OTHER,

  _NBS_TYPE
};

#endif // _STATUSMESSAGE_H
