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

    void initSafety(const int, const int, const int, const int);
    void initController(const int, const int, const int, const int);
    void initSensors(const int, const int, const int, const int);

    // Status Message
    void sendStatus(int level, int type, String msg);
    void sendStatus(int level, int type);
    void sendNotInit(int type);

    bool getEStopState();
    
    // Read actual EStop State
    void readDebouncedEStop();

    // Set EStop output
    void setEStop(bool);

    // Safety : Read forwarded estop
    void readForwardedEStop();
    
  private:
    void setCommonPins(const int, const int, const int, const int);
  
    bool currentEStopState;
    bool lastEStopState;

    int eStopPin;
    int eStopStatePin;

    // Safety : Forwarded pins
    int controllerEStopPin;
    int sensorEStopPin;

    long lastDebounceTime = 0;
    long delayDebounceInterval = 50;
};

#endif // _ERROR_HANDLER_H
