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
    sendStatusNotInitialize(FATAL, GPS_DEVICE);
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
    sendStatusNotInitialize(ERROR, GPS_DEVICE);
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
    sendStatusNotInitialize(ERROR, GPS_DEVICE);
    return -1;
  }
  return myGPS.getAltitude() - altitudeRef;
}

void Gps::updateLon()
{
  if (!setupDone)
  {
    sendStatusNotInitialize(ERROR, GPS_DEVICE);
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
    sendStatusNotInitialize(ERROR, GPS_DEVICE);
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
    sendStatusNotInitialize(ERROR, GPS_DEVICE);
    return -1;
  }  
  return myGPS.getSIV();
}

long Gps::getAlt()
{
  if (!setupDone)
  {
    sendStatusNotInitialize(ERROR, GPS_DEVICE);
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
    
    msg->data_count = NBS_DATA_SENT;
    msg->data[0] = getX();               //x
    msg->data[1] = getY();               //y
    msg->data[2] = getZ();               //z
    msg->data[3] = latitude;             //lat
    msg->data[4] = longitude;            //lon
    msg->data[5] = myGPS.getAltitude();  //alt
  }
  else
    sendStatusNotInitialize(ERROR, GPS_DEVICE);
 
}
