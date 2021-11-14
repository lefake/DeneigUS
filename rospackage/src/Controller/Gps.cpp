#include "Gps.h"

Gps::Gps() : setupDone(false) {}

Gps::~Gps(){}

void Gps::init()
{
  
  Wire.begin();
  setupDone = myGPS.begin();

  if(setupDone)
  {
    myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
    myGPS.saveConfiguration(); //Save the current settings to flash and BBR

    setZero();
  }
  else
    sendStatusNotInitialized(GPS_DEVICE);
}

void Gps::setZero()
{
  if(setupDone)
  {
    latitudeRef = myGPS.getLatitude();
    longitudeRef = myGPS.getLongitude();
    altitudeRef = myGPS.getAltitude();
  }
  else
    sendStatusNotInitialized(GPS_DEVICE);
}

long Gps::getX()
{
  return (longitude - longitudeRef)*7.821; //mm
}

long Gps::getY()
{
  return (latitude - latitudeRef)*11.132; //mm
}

long Gps::getZ()
{
  if (!setupDone)
  {
    sendStatusNotInitialized(GPS_DEVICE);
    return -1;
  }
  return myGPS.getAltitude() - altitudeRef;
}

void Gps::updateLon()
{
  if (!setupDone)
  {
    sendStatusNotInitialized(GPS_DEVICE);
    longitude = -1;
  }
  else
  {
    longitude = myGPS.getLongitude();
  }
}

void Gps::updateLat()
{
  if (!setupDone)
  {
    sendStatusNotInitialized(GPS_DEVICE);
    latitude = -1;
  }
  else
  {
    latitude = myGPS.getLatitude();
  }
}

long Gps::getSIVs()
{
  if (!setupDone)
  {
    sendStatusNotInitialized(GPS_DEVICE);
    return -1;
  }  
  return myGPS.getSIV();
}

long Gps::getAlt()
{
  if (!setupDone)
  {
    sendStatusNotInitialized(GPS_DEVICE);
    return -1;
  }
  return myGPS.getAltitude();
}

void Gps::getCoordinates( FloatArray* msg )
{  
  if (setupDone)
  {
    updateLon();
    updateLat();

    msg->data_count = 3;
    msg->data[0] = latitude;             //lat
    msg->data[1] = longitude;            //lon
    msg->data[2] = myGPS.getAltitude();  //alt
  }
  else
    sendStatusNotInitialized(GPS_DEVICE);
 
}
