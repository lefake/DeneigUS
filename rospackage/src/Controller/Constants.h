#ifndef _CONSTANTS_H
#define _CONSTANTS_H

// ======================= Sonars ===================================
#define NBS_SONARS                          2
#define MIN_DIST_DETECTION_M                0.02
#define MAX_DIST_DETECTION_M                2
#define TEMPERATURE                         20


// ======================= IMU ======================================
#define IMU_ADDRESS                         0x68
#define IMU_EEPROM_ADDRESS                  0x00
#define IMU_MAG_DECLINATION                 14.926      //https://www.geomag.nrcan.gc.ca/calc/mdcal-en.php

// ======================= GPS ======================================
#ifndef DEGTORAD
#define DEGTORAD                            0.0174532925199432957f
#define RADTODEG                            57.295779513082320876f
#endif

// ======================= SERIAL COMMUNICATION =====================
#define ID_BROADCAST_DELAY                  500
#define DATA_MSGS                           0
#define META_MSGS                           1

// ======================= Protobuf constants =======================

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

#endif // _CONSTANTS_H