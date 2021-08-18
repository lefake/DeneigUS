#ifndef IMU_H
#define IMU_H

#include <EEPROM.h>
// IMU Library by hideakitai 0.4.1
#include "MPU9250.h"
#include "twist.pb.h"
#include "constants.h"

class IMU {
  public:
    IMU();
    ~IMU();
    
    void init();
    void calibrateGyroAcc();
    void calibrateMag();
    void getValuesRos( Twist*, float );

  private:
    bool setup_done;
    MPU9250 imu;

    float vel_x;
    float pos_x;
    float vel_y;
    float pos_y;

    struct IMUCalibration {
      float acc_bias[3];
      float gyro_bias[3];
      float mag_bias[3];
      float mag_scale[3];
    };
};

#endif
