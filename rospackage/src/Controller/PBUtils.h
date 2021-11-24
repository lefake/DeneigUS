/*=============================================================================================================================
Author: Marc-Antoine C. Lafrenière
Project: DeneigUS
Date: April 2020
Description: Header file of the nanopb communication protocol 
             Encode and decode protobuf messages with an id from the serial port
             The messages are of the form : <id|msg;id|msg;> (can have from one to MAX_NBS_MSG messages)
==============================================================================================================================*/
#ifndef _PBUTILS_H
#define _PBUTILS_H

#include <pb_common.h>
#include <pb.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include "ErrorHandler.h"

#define MAX_MSG_LEN                         125
#define MAX_NBS_MSG                         6

struct Topic {
  int id;
  const pb_msgdesc_t* type;
  void* msg;
};

enum TOPICS
{ 
  // OUT
  DEBUG_ARDUINO = 0,
  ENC,
  IMU,
  GPS,
  SONAR_PAIRS,
  SOUFFLANTE_HEIGHT,
  ESTOP_STATE,
  DEBUG_MOT,

  // IN
  PROP,
  CHUTE,
  SOUFFLANTE_CMD,
  DEADMAN,
  ESTOP,
  LIGHT,
  PID_CST,

  _NBS_TOPICS
};

class PBUtils
{
  public:
    PBUtils(Topic*, ErrorHandler*);
    ~PBUtils();
    bool decodePb(char* , int *, int &);
    void pbSend(int, ...);

  private:
    int parseMsg(char[], int *, char **);
    void charsToBytes(char* , uint8_t*);
    uint8_t charToHex(char, char);
    
    pb_msgdesc_t *idToType[_NBS_TOPICS];
    void* idToMsg[_NBS_TOPICS];

    ErrorHandler* errorHandler;
};



#endif // _PBUTILS_H
