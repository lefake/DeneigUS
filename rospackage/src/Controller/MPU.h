#ifndef _MPU_H
#define _MPU_H

#include <EEPROM.h>
#include <Arduino.h>
#include "MPU9250.h"
#include "Constants.h"
#include "ErrorHandler.h"
#include "floatarray.pb.h"

class MPU
{
  public:
    MPU(ErrorHandler* e);
    
    void init();
    void getValues( FloatArray* );
    
    void doCalibration();
    void loadCalibration();
    void saveCalibration();
    void hardcodeCalibration();

  private:
    MPU9250 mpu;
    bool setupDone = false;
    ErrorHandler* errorHandler;

    struct IMUCalibration {
      float accBias[3];
      float gyroBias[3];
      float magBias[3];
      float magScale[3];
    };
};

MPU::MPU(ErrorHandler* e) 
{
  errorHandler = e;
}

void MPU::init()
{
  Wire.begin();
  delay(1000);
  
  setupDone = mpu.setup(IMU_ADDRESS);
  if (!setupDone)
    errorHandler->sendNotInit(IMU_DEVICE);
}

void MPU::getValues(FloatArray* msg)
{
  if (mpu.update())
  {
    msg->data_count = 4;
    msg->data[0] = mpu.getQuaternionX();
    msg->data[1] = mpu.getQuaternionY();
    msg->data[2] = mpu.getQuaternionZ();
    msg->data[3] = mpu.getQuaternionW();
  }
  else
    errorHandler->sendStatus(ERROR, IMU_DEVICE, "Unable to update IMU values");
}

void MPU::saveCalibration() {
  errorHandler->sendStatus(DEBUG, IMU_DEVICE, "Writing config to EEPROM");
  
  IMUCalibration calibration = {
    .accBias = {mpu.getAccBias(0), mpu.getAccBias(1), mpu.getAccBias(2)},
    .gyroBias = {mpu.getGyroBias(0), mpu.getGyroBias(1), mpu.getGyroBias(2)},
    .magBias = {mpu.getMagBias(0), mpu.getMagBias(1), mpu.getMagBias(2)},
    .magScale = {mpu.getMagScale(0), mpu.getMagScale(1), mpu.getMagScale(2)}
  };
  EEPROM.put(IMU_EEPROM_ADDRESS, calibration);
  
  errorHandler->sendStatus(DEBUG, IMU_DEVICE, "Done writing config to EEPROM");
}

void MPU::loadCalibration() {
  errorHandler->sendStatus(DEBUG, IMU_DEVICE, "Reading config to EEPROM");

  IMUCalibration calibration;
  EEPROM.get(IMU_EEPROM_ADDRESS, calibration);
  
  mpu.setAccBias(calibration.accBias[0], calibration.accBias[1], calibration.accBias[2]);
  mpu.setGyroBias(calibration.gyroBias[0], calibration.gyroBias[1], calibration.gyroBias[2]);
  mpu.setMagBias(calibration.magBias[0], calibration.magBias[1], calibration.magBias[2]);
  mpu.setMagScale(calibration.magScale[0], calibration.magScale[1], calibration.magScale[2]);
  mpu.setMagneticDeclination(IMU_MAG_DECLINATION);
  
  errorHandler->sendStatus(DEBUG, IMU_DEVICE, "Done reading config to EEPROM");
}

void MPU::hardcodeCalibration()
{
  mpu.setAccBias(-3553.30, 4414.70, -3685.60);
  mpu.setGyroBias(659.65, -4.80, -3407.55);
  mpu.setMagBias(198.29, 448.69, 238.54);
  mpu.setMagScale(1.09, 0.74, 1.39);
  mpu.setMagneticDeclination(IMU_MAG_DECLINATION);
}

void MPU::doCalibration()
{
   // calibrate anytime you want to
  errorHandler->sendStatus(INFO, IMU_DEVICE, "Please leave the device still on the flat plane.");
  delay(1000);
  mpu.calibrateAccelGyro();

  errorHandler->sendStatus(INFO, IMU_DEVICE, "Please Wave device in a figure eight until done.");
  
  delay(2000);
  mpu.calibrateMag();
  saveCalibration();
  loadCalibration();

  errorHandler->sendStatus(INFO, IMU_DEVICE, "Calibration done.");
}

#endif //_MPU_H
