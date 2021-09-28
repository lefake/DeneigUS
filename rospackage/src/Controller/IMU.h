#ifndef _IMU_H
#define _IMU_H

#include <EEPROM.h>
// IMU Library by hideakitai 0.4.1
#include "MPU9250.h"
#include "StatusMessage.h"
#include "twist.pb.h"
#include "Constants.h"

class IMU {
  public:
    IMU();
    
    void init();
    void getValues( Twist*, float );

  private:
    bool setupDone;
    MPU9250 imu;

    float velX;
    float posX;
    float velY;
    float posY;

    struct IMUCalibration {
      float accBias[3];
      float gyroBias[3];
      float magBias[3];
      float magScale[3];
    };
};

#endif // _IMU_H
