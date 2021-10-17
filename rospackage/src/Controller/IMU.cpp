#include "IMU.h"

IMU::IMU() : velX(0), posX(0), velY(0), posY(0) {
  Wire.begin();
  setupDone = false;
}

void IMU::init()
{
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


void IMU::getValues( Twist* msg, float periodMs )
{
  if (setupDone)
  {
    imu.update();
    velX += imu.getLinearAccX() * (periodMs / 1000.0);
    posX += velX * (periodMs / 1000.0);
    velY += imu.getLinearAccY() * (periodMs / 1000.0);
    posY += velY * (periodMs / 1000.0);

    msg->lx = posX/1000;
    msg->ly = posY/1000;
    msg->az = imu.getGyroZ()/180*3.145;
  }
  else
    sendStatusNotInitialized(IMU_DEVICE);
}
