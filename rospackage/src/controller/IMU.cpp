#include "IMU.h"

IMU::IMU() {
  Wire.begin();
  setup_done = false;
}

IMU::~IMU() { }

void IMU::init()
{
  if (imu.setup(IMU_ADDRESS))
    setup_done = true;
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
