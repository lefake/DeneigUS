#ifndef _MPU_H
#define _MPU_H

#include <EEPROM.h>
#include <Arduino.h>
#include "MPU9250.h"
#include "Constants.h"
#include "StatusMessage.h"
#include "floatarray.pb.h"

class MPU
{
  public:
    MPU();
    
    void init();
    void getValues( FloatArray* );
    
    void doCalibration();
    void loadCalibration();
    void saveCalibration();

  private:
    MPU9250 mpu;
    bool setupDone = false;
    
    void writeByte(int address, byte value);
    void writeFloat(int address, float value);
    byte readByte(int address);
    float readFloat(int address);
    void clearCalibration();
    bool isCalibrated();

    enum EEP_ADDR {
      EEP_CALIB_FLAG = 0x00,
      EEP_ACC_BIAS = 0x01,
      EEP_GYRO_BIAS = 0x0D,
      EEP_MAG_BIAS = 0x19,
      EEP_MAG_SCALE = 0x25
    };
};

MPU::MPU() { }

void MPU::init()
{
  Wire.begin();
  delay(1000);
  
  setupDone = mpu.setup(IMU_ADDRESS);
  if (setupDone)
    loadCalibration();
  else
    sendStatusNotInitialized(IMU_DEVICE);
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
    sendStatusWithMessage(ERROR, IMU_DEVICE, "Unable to update IMU values");
}

void MPU::saveCalibration() {
    sendStatusWithMessage(DEBUG, IMU_DEVICE, "Writing config to EEPROM");
    
    writeByte(EEP_CALIB_FLAG, 1);
    writeFloat(EEP_ACC_BIAS + 0, mpu.getAccBias(0));
    writeFloat(EEP_ACC_BIAS + 4, mpu.getAccBias(1));
    writeFloat(EEP_ACC_BIAS + 8, mpu.getAccBias(2));
    writeFloat(EEP_GYRO_BIAS + 0, mpu.getGyroBias(0));
    writeFloat(EEP_GYRO_BIAS + 4, mpu.getGyroBias(1));
    writeFloat(EEP_GYRO_BIAS + 8, mpu.getGyroBias(2));
    writeFloat(EEP_MAG_BIAS + 0, mpu.getMagBias(0));
    writeFloat(EEP_MAG_BIAS + 4, mpu.getMagBias(1));
    writeFloat(EEP_MAG_BIAS + 8, mpu.getMagBias(2));
    writeFloat(EEP_MAG_SCALE + 0, mpu.getMagScale(0));
    writeFloat(EEP_MAG_SCALE + 4, mpu.getMagScale(1));
    writeFloat(EEP_MAG_SCALE + 8, mpu.getMagScale(2));
}

void MPU::loadCalibration() {
    sendStatusWithMessage(DEBUG, IMU_DEVICE, "Reading config to EEPROM");
    if (isCalibrated()) 
    {
        mpu.setAccBias(
            readFloat(EEP_ACC_BIAS + 0),
            readFloat(EEP_ACC_BIAS + 4),
            readFloat(EEP_ACC_BIAS + 8));
        mpu.setGyroBias(
            readFloat(EEP_GYRO_BIAS + 0),
            readFloat(EEP_GYRO_BIAS + 4),
            readFloat(EEP_GYRO_BIAS + 8));
        mpu.setMagBias(
            readFloat(EEP_MAG_BIAS + 0),
            readFloat(EEP_MAG_BIAS + 4),
            readFloat(EEP_MAG_BIAS + 8));
        mpu.setMagScale(
            readFloat(EEP_MAG_SCALE + 0),
            readFloat(EEP_MAG_SCALE + 4),
            readFloat(EEP_MAG_SCALE + 8));
    } 
    else 
    {
        mpu.setAccBias(0., 0., 0.);
        mpu.setGyroBias(0., 0., 0.);
        mpu.setMagBias(0., 0., 0.);
        mpu.setMagScale(1., 1., 1.);
    }
}

void MPU::doCalibration()
{
     // calibrate anytime you want to
    sendStatusWithMessage(INFO, IMU_DEVICE, "Accel Gyro calibration will start in 5sec.");
    sendStatusWithMessage(INFO, IMU_DEVICE, "Please leave the device still on the flat plane.");
    delay(1000);
    mpu.calibrateAccelGyro();

    sendStatusWithMessage(INFO, IMU_DEVICE, "Mag calibration will start in 5sec.");
    sendStatusWithMessage(INFO, IMU_DEVICE, "Please Wave device in a figure eight until done.");
    delay(2000);
    mpu.calibrateMag();
    saveCalibration();
    loadCalibration();
}

void MPU::writeByte(int address, byte value) {
    EEPROM.put(address, value);
}

void MPU::writeFloat(int address, float value) {
    EEPROM.put(address, value);
}

byte MPU::readByte(int address) {
    byte valueIn = 0;
    EEPROM.get(address, valueIn);
    return valueIn;
}

float MPU::readFloat(int address) {
    float valueIn = 0;
    EEPROM.get(address, valueIn);
    return valueIn;
}

void MPU::clearCalibration() {
    writeByte(EEP_CALIB_FLAG, 0);
}

bool MPU::isCalibrated() {
    return (readByte(EEP_CALIB_FLAG) == 0x01);
}

#endif //_MPU_H
