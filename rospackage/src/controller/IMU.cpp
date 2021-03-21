#include "IMU.h"

IMU::IMU() {
  Wire.begin();
  setup_done = false;
}

IMU::~IMU() { }

void IMU::init()
{
  setup_done = imu.setup(IMU_ADDRESS);
  
  if (setup_done)
  {
    imu.setAccBias(IMU_ACC_BIAS_X, IMU_ACC_BIAS_Y, IMU_ACC_BIAS_Z);
    imu.setGyroBias(IMU_GYRO_BIAS_X, IMU_GYRO_BIAS_Y, IMU_GYRO_BIAS_Z);
    imu.setMagBias(IMU_MAG_BIAS_X, IMU_MAG_BIAS_Y, IMU_MAG_BIAS_Z);
    imu.setMagScale(IMU_MAG_SCALE_X, IMU_MAG_SCALE_Y, IMU_MAG_SCALE_Z);
    imu.setMagneticDeclination(IMU_MAG_DECLINATION);
  }
}

void IMU::calibrate()
{
  if (setup_done)
  {
    imu.calibrateAccelGyro();
    imu.calibrateMag(); 
  }
}

void IMU::getValuesRos( std_msgs::Float32MultiArray* msg )
{
  if (setup_done)
  {
    imu.update();

    for (int i = 0; i < 3; i++)
    {
      msg->data[i] = imu.getGyro(i);
      msg->data[3 + i] = imu.getAcc(i);
      msg->data[6 + i] = imu.getMag(i);
    }
  }
  else 
  {
    for (int i = 0; i < IMU_DATA_MSG_ARRAY_LEN; i++)
    {
      msg->data[i] = -1;
    }
  }
}
