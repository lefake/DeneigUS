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

#include <Arduino.h>
#include <pb_common.h>
#include <pb.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include "StatusMessage.h"

#define MAX_MSG_LEN                         500
#define MAX_NBS_MSG                         10

struct Topic {
  int id;
  pb_msgdesc_t* type;
  void* msg;
};

class PBUtils
{
  public:
    PBUtils(Topic*, int);
    ~PBUtils();
    bool decodePb(char* , int *, int &);
    void pbSend(int, ...);

  private:
    int parseMsg(char[], int *, char **);
    void charsToBytes(char* , uint8_t*);
    uint8_t charToHex(char, char);
    
    pb_msgdesc_t** idToType;
    void** idToMsg;
};

#endif // _PBUTILS_H
