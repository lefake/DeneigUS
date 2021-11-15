#ifndef _IMU_H
#define _IMU_H

#include <EEPROM.h>
#include <Arduino.h>
#include "MPU9250.h"
#include "Constants.h"
#include "StatusMessage.h"
#include "floatarray.pb.h"

class IMU {
  public:
    IMU();
    
    void init();
    void getValues( FloatArray*);

  private:
    bool setupDone;
    MPU9250 imu;

    struct IMUCalibration {
      float accBias[3];
      float gyroBias[3];
      float magBias[3];
      float magScale[3];
    };
};

#endif // _IMU_H
