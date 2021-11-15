#include "IMU.h"

IMU::IMU() {
  setupDone = false;
}

void IMU::init()
{
  Wire.begin();
  setupDone = imu.setup(IMU_ADDRESS);
  
  if (setupDone)
  {
    IMUCalibration calibration;
    EEPROM.get(IMU_EEPROM_ADDRESS, calibration);
    
    imu.setAccBias(calibration.accBias[0], calibration.accBias[1], calibration.accBias[2]);
    imu.setGyroBias(calibration.gyroBias[0], calibration.gyroBias[1], calibration.gyroBias[2]);
    imu.setMagBias(calibration.magBias[0], calibration.magBias[1], calibration.magBias[2]);
    imu.setMagScale(calibration.magScale[0], calibration.magScale[1], calibration.magScale[2]);
    imu.setMagneticDeclination(IMU_MAG_DECLINATION);
  }
  else
    sendStatusNotInitialized(IMU_DEVICE);
}


void IMU::getValues( FloatArray* msg )
{
  if (setupDone)
  {
    imu.update();
    msg->data_count = 4;
    msg->data[0] = imu.getQuaternionX();
    msg->data[1] = imu.getQuaternionY();
    msg->data[2] = imu.getQuaternionZ();
    msg->data[3] = imu.getQuaternionW();
  }
  else
    sendStatusNotInitialized(IMU_DEVICE);
}
