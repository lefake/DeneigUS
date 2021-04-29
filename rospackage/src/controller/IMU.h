#ifndef IMU_H
#define IMU_H

// IMU Library by hideakitai 0.4.1
#include "MPU9250.h"
#include "floatarray.pb.h"
#include "constants.h"

class IMU {
  public:
    IMU();
    ~IMU();
    
    void init();
    void calibrateGyroAcc();
    void calibrateMag();
    void getValuesRos( FloatArray*, float );

  private:
    bool setup_done;
    MPU9250 imu;

    float vel_x;
    float pos_x;
    float vel_y;
    float pos_y;
};

#endif
