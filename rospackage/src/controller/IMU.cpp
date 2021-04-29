#include "IMU.h"

IMU::IMU() : vel_x(0), pos_x(0), vel_y(0), pos_y(0) {
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

void IMU::calibrateGyroAcc()
{
  if (setup_done)
    imu.calibrateAccelGyro();
}

void IMU::calibrateMag()
{
  if (setup_done)
    imu.calibrateMag(); 
}

void IMU::getValuesRos( FloatArray* msg, float period_ms )
{
  if (setup_done)
  {
    imu.update();
    vel_x += imu.getLinearAccX() * (period_ms / 1000.0);
    pos_x += vel_x * (period_ms / 1000.0);
    vel_y += imu.getLinearAccY() * (period_ms / 1000.0);
    pos_y += vel_y * (period_ms / 1000.0);
    
    msg->data[0] = pos_x;
    msg->data[1] = pos_y;
    msg->data[2] = imu.getQuaternionX();
    msg->data[3] = imu.getQuaternionY();
    msg->data[4] = imu.getQuaternionZ();
    msg->data[5] = imu.getQuaternionW();
    
    /*
    
    for (int i = 0; i < 3; i++)
    {
      msg->data[i] = imu.getGyro(i);
      msg->data[3 + i] = imu.getAcc(i);
      msg->data[6 + i] = imu.getMag(i);
    }*/
  }
  else 
  {
    for (int i = 0; i < IMU_DATA_MSG_ARRAY_LEN; i++)
    {
      msg->data[i] = -1;
    }
  }
}
