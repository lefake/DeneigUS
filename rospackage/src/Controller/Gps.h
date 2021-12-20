#ifndef _GPS_H
#define _GPS_H

#include <Wire.h>
#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS
#include "ErrorHandler.h"
#include "floatarray.pb.h"
#include "Constants.h"

class Gps
{
  public:
    Gps(ErrorHandler* e);
    ~Gps();
    void init();
    void getCoordinates( FloatArray* msg );

  private:
    SFE_UBLOX_GPS myGPS;
    bool setupDone = false;
    ErrorHandler* errorHandler;
};

#endif // _GPS_H
