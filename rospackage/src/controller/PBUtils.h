/*=============================================================================================================================
Author: Marc-Antoine C. Lafrenière
Project: DeneigUS
Date: April 2020
Description: Header file of the nanopb communication protocol 
             Encode and decode protobuf messages with an id on the serial port
             The messages are of the form : <id|msg;id|msg;> (can have from one to MAX_NBS_MSG messages)
==============================================================================================================================*/
#ifndef PBUTILS_H
#define PBUTILS_H

#include <Arduino.h>
#include <pb_common.h>
#include <pb.h>
#include <pb_encode.h>
#include <pb_decode.h>

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
    bool decode_pb(char* , int *, int &);
    bool pb_send(int* );

  private:
    int parse_msg(char[], int *, char **);
    void chars2bytes(char* , uint8_t*);
    uint8_t char2hex(char, char);
    
    pb_msgdesc_t** _id_2_type;
    void** _id_2_msg;
    uint8_t dict['f'-'0'+1];
};

#endif
