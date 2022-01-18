#include "Gps.h"

Gps::Gps() {}

Gps::~Gps(){}

void Gps::init()
{
  Wire.begin();
  setupDone = myGPS.begin();

  if(setupDone)
  {
    myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
    myGPS.saveConfiguration();
  }
  else
    sendStatusNotInitialized(GPS_DEVICE);
}

void Gps::getCoordinates( FloatArray* msg )
{  
  if (setupDone)
  {
    msg->data_count = 3;
    msg->data[0] = myGPS.getLatitude();
    msg->data[1] = myGPS.getLongitude();
    msg->data[2] = myGPS.getAltitude();
  }
  else
    sendStatusNotInitialized(GPS_DEVICE);
}
