#ifndef PBCONSTANTS_H
#define PBCONSTANTS_H

#include "msg/deneigus/twist.pb.h"
#include "msg/deneigus/floatarray.pb.h"

#define NBS_MSG_TYPES           2

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

enum TYPE
{
  TWIST = 0,
  FLOATARRAY,
};

extern const pb_msgdesc_t* MSGS_TYPE[NBS_MSG_TYPES];
extern const int TOPIC_2_MSG_TYPE[_NBS_TOPICS];

#endif
