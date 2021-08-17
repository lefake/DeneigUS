#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <pb.h>

// ----------------------- ROS messages constants -----------------------
#define NBS_SONARS                          2
#define NBS_BATTERIES                       2
#define NBS_MOTORS                          5
#define NBS_GPS                             1
#define NBS_IMU                             1

#define NBS_DATA_POS                        3
#define NBS_DATA_SONRARS                    2
#define NBS_DATA_ESTOP                      1
#define NBS_DATA_BATTERIES                  4
#define NBS_DATA_TOURELLES                  5
#define NBS_DATA_MOTORS                     5
#define NBS_DATA_GPS                        6
#define NBS_DATA_IMU                        6

#define POS_MSG_ARRAY_LEN                   NBS_DATA_POS
#define OBS_POS_MSG_ARRAY_LEN               NBS_DATA_SONRARS * NBS_SONARS
#define ESTOP_STATE_MSG_ARRAY_LEN           NBS_DATA_ESTOP
#define TELE_BATT_MSG_ARRAY_LEN             NBS_DATA_BATTERIES * NBS_BATTERIES
#define POS_TOURELLE_MSG_ARRAY_LEN          NBS_DATA_TOURELLES
#define DEBUG_MOT_MSG_ARRAY_LEN             NBS_DATA_MOTORS * NBS_MOTORS
#define GPS_DATA_MSG_ARRAY_LEN              NBS_DATA_GPS * NBS_GPS
#define IMU_DATA_MSG_ARRAY_LEN              NBS_DATA_IMU * NBS_IMU
#define DEBUG_ARDUINO_DATA_MSG_ARRAY_LEN    OBS_POS_MSG_ARRAY_LEN


// ----------------------- Sonars constants -----------------------
#define MIN_DIST_DETECTION_M                0.02
#define MAX_DIST_DETECTION_M                2

#define TEMPERATURE                         20


// ----------------------- IMU contants ----------------------- 
#define IMU_ADDRESS                         0x68
#define IMU_EEPROM_ADDRESS                  0x00
#define IMU_MAG_DECLINATION                 14.926      //https://www.geomag.nrcan.gc.ca/calc/mdcal-en.php

// ----------------------- GPS -----------------------
#ifndef DEGTORAD
#define DEGTORAD                            0.0174532925199432957f
#define RADTODEG                            57.295779513082320876f
#endif

// ----------------------- Protobuf constants -----------------------

enum TOPICS
{
  DEBUG_ARDUINO = 0,
  CMD_VEL,
  CMD_TOURELLE,
  POS,
  OBS_POS,
  ESTOP_STATE,
  TELE_BATT,
  POS_TOURELLE,
  DEBUG_MOT,
  GPS_DATA,
  IMU_DATA,

  _NBS_TOPICS
};

#endif
