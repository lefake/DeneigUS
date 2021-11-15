#ifndef _GPS_H
#define _GPS_H

#include <Wire.h>
#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS
#include "StatusMessage.h"
#include "floatarray.pb.h"
#include "Constants.h"

class Gps
{
  public:
    Gps();
    ~Gps();
    void init();
    void getCoordinates( FloatArray* msg );

  private:
    SFE_UBLOX_GPS myGPS;
    bool setupDone = false;

};

#endif // _GPS_H
