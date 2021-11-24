#ifndef _ERROR_HANDLER_H
#define _ERROR_HANDLER_H

#include <Arduino.h>

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

const int LOG_LEVEL = INFO;

class ErrorHandler
{
  public:
    ErrorHandler();
    ~ErrorHandler();

    void init();
  
    void sendStatus(int level, int type, String msg);
    void sendStatus(int level, int type);
    void sendNotInit(int type);

    void setEStop(bool);
    bool readEStop();
    int readDebouncedEStop();
    bool getEStop();
    
  private:
    bool eStopState;
    bool lastEStopState;

    long lastDebounceTime = 0;
    long delayDebounceInterval = 50;
    
    const int eStopStatePin;
    const int eStopPin;
};

#endif // _ERROR_HANDLER_H
