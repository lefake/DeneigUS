#include "Gps.h"

Gps::Gps(ErrorHandler* e) 
{
  errorHandler = e;
}

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
    errorHandler->sendNotInit(GPS_DEVICE);
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
    errorHandler->sendNotInit(GPS_DEVICE);
}
