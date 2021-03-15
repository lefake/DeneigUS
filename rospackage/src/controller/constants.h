#ifndef CONSTANTS_H
#define CONSTANTS_H

#define NBS_SONARS                          6
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
#define NBS_DATA_IMU                        9

#define POS_MSG_ARRAY_LEN                   NBS_DATA_POS
#define OBS_POS_MSG_ARRAY_LEN               NBS_DATA_SONRARS * NBS_SONARS
#define ESTOP_STATE_MSG_ARRAY_LEN           NBS_DATA_ESTOP
#define TELE_BATT_MSG_ARRAY_LEN             NBS_DATA_BATTERIES * NBS_BATTERIES
#define POS_TOURELLE_MSG_ARRAY_LEN          NBS_DATA_TOURELLES
#define DEBUG_MOT_MSG_ARRAY_LEN             NBS_DATA_MOTORS * NBS_MOTORS
#define GPS_DATA_MSG_ARRAY_LEN              NBS_DATA_GPS * NBS_GPS
#define IMU_DATA_MSG_ARRAY_LEN              NBS_DATA_IMU * NBS_IMU
#define DEBUG_ARDUINO_DATA_MSG_ARRAY_LEN    OBS_POS_MSG_ARRAY_LEN


// Sonars constants
#define MIN_DIST_DETECTION_CM               2
#define MAX_DIST_DETECTION_CM               400
#define DIST_THRESHOLD_CM                   20

#define TEMPERATURE                         20


//IMU contants
#define IMU_ADDRESS                         0x68


// GPS
#ifndef DEGTORAD
#define DEGTORAD 0.0174532925199432957f
#define RADTODEG 57.295779513082320876f
#endif

#endif
