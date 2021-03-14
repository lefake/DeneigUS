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
  
}

void Gps::setZero()
{
  if(setup_done)
  {
    latitudeRef = myGPS.getLatitude();
    longitudeRef = myGPS.getLongitude();
    altitudeRef = myGPS.getAltitude();
  }
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
  return (setup_done) ? myGPS.getAltitude() - altitudeRef : -1;
}

long Gps::updateLon()
{
  longitude = (setup_done) ? myGPS.getLongitude() : -1;
}

long Gps::updateLat()
{
  latitude = (setup_done) ? myGPS.getLatitude() : -1;
}

long Gps::getSIVs()
{
  return (setup_done) ? myGPS.getSIV() : -1;
}

long Gps::getAlt()
{
  return (setup_done) ? myGPS.getAltitude() : -1;
}

void Gps::getCoordinates( std_msgs::Float32MultiArray* msg )
{  
  if (setup_done)
  {
    updateLon();
    updateLat();
    
    msg->data[0] = getX() / 1000.0;               //x
    msg->data[1] = getY() / 1000.0;               //y
    msg->data[2] = getZ() / 1000.0;               //z
    msg->data[3] = latitude;                      //lat
    msg->data[4] = longitude;                     //lon
    msg->data[5] = myGPS.getAltitude();           //alt
  }
  else
  {
    for (int i = 0; i < GPS_DATA_MSG_ARRAY_LEN; i++)
    {
      msg->data[i] = i;
    }
  }
 
}
