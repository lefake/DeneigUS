#ifndef _GPS_H
#define _GPS_H

#include <Wire.h>
#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS
#include "StatusMessage.h"
#include "floatarray.pb.h"
#include "Constants.h"

#define NBS_DATA_SENT           6

class Gps
{
  public:
    Gps();
    ~Gps();
    void init();
    void setZero();
    long getX();
    long getY();
    long getZ();
    void updateLon();
    void updateLat();
    long getSIVs();
    long getAlt();
    void getCoordinates( FloatArray* msg );

  private:
    SFE_UBLOX_GPS myGPS;
    long latitudeRef = 0;
    long longitudeRef = 0;
    long altitudeRef = 0;
    
    long latitude = 0;
    long longitude = 0;

    bool setupDone;

};

#endif // _GPS_H
