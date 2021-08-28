#include "Gps.h"

Gps::Gps(){ setup_done = false; }

Gps::~Gps(){}

void Gps::init()
{
  
  Wire.begin();
  setup_done = myGPS.begin();

  if(setup_done)
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
  if(setup_done)
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
  if (!setup_done)
    sendStatusNotInitialize(ERROR, GPS_DEVICE);
  return (longitude - longitudeRef)*7.821; //mm
}

long Gps::getY()
{
  if (!setup_done)
    sendStatusNotInitialize(ERROR, GPS_DEVICE);
  return (latitude - latitudeRef)*11.132; //mm
}

long Gps::getZ()
{
  if (!setup_done)
    sendStatusNotInitialize(ERROR, GPS_DEVICE);
  return (setup_done) ? myGPS.getAltitude() - altitudeRef : -1;
}

void Gps::updateLon()
{
  if (!setup_done)
    sendStatusNotInitialize(ERROR, GPS_DEVICE);
  longitude = (setup_done) ? myGPS.getLongitude() : -1;
}

void Gps::updateLat()
{
  latitude = (setup_done) ? myGPS.getLatitude() : -1;
}

long Gps::getSIVs()
{
  if (!setup_done)
    sendStatusNotInitialize(ERROR, GPS_DEVICE);
  return (setup_done) ? myGPS.getSIV() : -1;
}

long Gps::getAlt()
{
  if (!setup_done)
    sendStatusNotInitialize(ERROR, GPS_DEVICE);
  return (setup_done) ? myGPS.getAltitude() : -1;
}

void Gps::getCoordinates( FloatArray* msg )
{  
  if (setup_done)
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
