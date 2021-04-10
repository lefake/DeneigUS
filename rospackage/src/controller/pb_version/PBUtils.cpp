#include "PBUtils.h"

bool PBUtils::decode_pb(char* inputString, void *msgs_in[_NBS_TOPICS-1], int *sub_msg_id, int &nbs_new_msgs)
{
  char *sub_msgs[MAX_MSG_LEN];
  bool success = true;
  
  nbs_new_msgs = parse_msg(inputString, sub_msg_id, sub_msgs);

  for (int i = 0; i < nbs_new_msgs; ++i)
  {
    empty_buffers();
    chars2bytes(sub_msgs[i], buffer_in);
  
    pb_istream_t stream_in = pb_istream_from_buffer(buffer_in, strlen(sub_msgs[i])/2);
    success = success && pb_decode(&stream_in, MSGS_TYPE[TOPIC_2_MSG_TYPE[sub_msg_id[i]]], msgs_in[sub_msg_id[i]]);
  }
  
  return success;
}

bool PBUtils::pb_send(int* ids, void *msgs_out[_NBS_TOPICS])
{
  bool success = true;
  int nbs_to_send = sizeof(ids)/sizeof(ids[0]);
  String to_send_builder = "<";

  Serial.print("(");
  Serial.print(ids[0]);
  Serial.println(")");

  for (int i = 0; i < nbs_to_send; ++i)
  {
    empty_buffers();
    pb_ostream_t stream = pb_ostream_from_buffer(buffer_out, sizeof(buffer_out));
    success = success && pb_encode(&stream, MSGS_TYPE[TOPIC_2_MSG_TYPE[ids[i]]], msgs_out[ids[i]]);

    if(success)
    {
      to_send_builder += String(ids[i]);
      to_send_builder += "|";
      
      for(int j = 0; j < stream.bytes_written; j++)
      {
        sprintf (to_send, "%02X", buffer_out[j]);
        to_send_builder += String(to_send);
      }
    }
    
    to_send_builder += ";";
  }
  
  to_send_builder += ">";

  if (success)
    Serial.print(to_send_builder);
  
  return success;
}

int PBUtils::parse_msg(char inString[], int *msg_ids, char **outStrings)
{
  // Split all messages
  char *whole_msgs[MAX_MSG_LEN];
  int ind = 0;
  char* substr = strtok(inString, ";");

  while (substr != NULL)
  {
    whole_msgs[ind++] = substr;
    substr = strtok(NULL, ";");
  }

  // Split (id, msg) pair
  for (int i = 0; i < ind; i++)
  {
    char* id = strtok(whole_msgs[i], "|");
    char* msg = strtok(NULL, "|");
    
    msg_ids[i] = atoi(id);
    outStrings[i] = msg;
  }

  return ind;
}

void PBUtils::chars2bytes(char* mystring, uint8_t* myuint)
{
  for (uint8_t i=0; i<strlen(mystring)/2;i++) {
      char chars[2] = { mystring[(i*2)], mystring[(i*2)+1] };
      myuint[i] = char2hex(chars);
  }
}

uint8_t PBUtils::char2hex(char in[2])
{
  uint8_t val[2];

  for (int i = 0; i < 2; i++)
  {
    if (in[i] >= 'A' && in[i] <= 'F')
      val[i] = in[i] - 'A' + 10;
    else if (in[i] >= 'a' && in[i] <= 'f')
      val[i] = in[i] - 'a' + 10;
    else
      val[i] = in[i] - '0';
  }
 
  return val[0]*16 + val[1];
}

void PBUtils::empty_buffers()
{
  for (int i = 0; i < MAX_MSG_LEN; ++i)
  {
    buffer_in[i] = 0;
    buffer_out[i] = 0;
    to_send[i] = '\0';
  }
}
