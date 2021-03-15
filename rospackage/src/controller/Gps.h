#ifndef GPS_H
#define GPS_H

#include <Wire.h>
#include "SparkFun_Ublox_Arduino_Library.h" //http://librarymanager/All#SparkFun_Ublox_GPS
#include <std_msgs/Float32MultiArray.h>
#include "constants.h"

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
    long updateLon();
    long updateLat();
    long getSIVs();
    long getAlt();
    void getCoordinates( std_msgs::Float32MultiArray* msg );

  private:
    SFE_UBLOX_GPS myGPS;
    long latitudeRef = 0;
    long longitudeRef = 0;
    long altitudeRef = 0;
    
    long latitude = 0;
    long longitude = 0;

    bool setup_done;

};

#endif // SONARS_H
