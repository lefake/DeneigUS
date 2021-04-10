#ifndef PBUTILS_H
#define PBUTILS_H

#include <Arduino.h>
#include <pb_common.h>
#include <pb.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include "constants.h"
#include "PBConstants.h"

#include "msg/deneigus/twist.pb.h"
#include "msg/deneigus/floatarray.pb.h"

class PBUtils
{
  public: 
    bool decode_pb(char* inputString, void *msgs_in[_NBS_TOPICS], int *sub_msg_id, int &nbs_new_msgs);
    bool pb_send(int* ids, void *msgs_out[_NBS_TOPICS]);

  private:
    
    uint8_t buffer_in[MAX_MSG_LEN];
    uint8_t buffer_out[MAX_MSG_LEN];
    char to_send[MAX_MSG_LEN];


    int parse_msg(char inString[], int *msg_ids, char **outStrings);
    void chars2bytes(char* mystring, uint8_t* myuint);
    uint8_t char2hex(char in[2]);
    void empty_buffers();
    
};

#endif
