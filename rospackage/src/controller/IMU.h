#ifndef IMU_H
#define IMU_H

#include "MPU9250.h"
#include "floatarray.pb.h"
#include "constants.h"

class IMU {
  public:
    IMU();
    ~IMU();
    
    void init();
    void calibrate();
    void getValuesRos( FloatArray* msg );

  private:
    bool setup_done;
    MPU9250 imu;
};

#endif
