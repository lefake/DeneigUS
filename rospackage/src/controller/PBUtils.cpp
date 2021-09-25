/*=============================================================================================================================
Author: Marc-Antoine C. Lafrenière
Project: DeneigUS
Date: April 2020
==============================================================================================================================*/
#include "PBUtils.h"

/*
 * Constructor of the class
 * 
 * @param topics: Array of all the topics containing their type
 * @param nbs_topics: Number of topic in the array
 */
PBUtils::PBUtils(Topic *topics, int nbs_topics)
{
  _id_2_type = malloc(sizeof(pb_msgdesc_t *) * nbs_topics);
  _id_2_msg = malloc(sizeof(void *) * nbs_topics);
  
  // Initializie array from the struct so it isn't dependant on array indexes
  // Hence the id must be positive and can't appear more than once
  // Ids must be valid array index. If there's 4 topics then id must be [0, 5[
  for (int i = 0; i < nbs_topics; ++i)
  {
    _id_2_type[topics[i].id] = topics[i].type;
    _id_2_msg[topics[i].id] = topics[i].msg;
  }
}

PBUtils::~PBUtils() 
{
  free(_id_2_type);
  free(_id_2_msg);
}

/*
 * Convert a string to a list of PB messages
 * 
 * @param input_string: The input str to decode from
 * @param sub_msg_id: An output list of all the messages id in the full message
 * @param nbs_new_msgs: The number of sub messages in the fill message
 * 
 * @return: if the decode was successful
 */
bool PBUtils::decode_pb(char* input_string, int *sub_msg_id, int &nbs_new_msgs)
{
  char *sub_msgs[MAX_MSG_LEN];
  bool success = true;
  
  nbs_new_msgs = parse_msg(input_string, sub_msg_id, sub_msgs);

  // Add assert nbs_new_msgs < length of sub_msg_id

  for (int i = 0; i < nbs_new_msgs; ++i)
  {
    uint8_t buffer_in[MAX_MSG_LEN];
    chars2bytes(sub_msgs[i], buffer_in);

    pb_istream_t stream_in = pb_istream_from_buffer(buffer_in, strlen(sub_msgs[i])/2);
    success = success && pb_decode(&stream_in, _id_2_type[sub_msg_id[i]], _id_2_msg[sub_msg_id[i]]);
  }
  
  return success;
}

/*
 * Send protobufs messages to the serial port with format <id|msg;>
 * 
 * @param nbs: The numbers of id to send
 * @param ...: List of all the ids to send 
 *             ex : cant use it like this pb_send(3, POS, OBS_POS, DEBUG_ARDUINO);
 * 
 * @return: If the encode was successful  
 */
bool PBUtils::pb_send(int nbs, ...)
{
  bool success = true;
  String to_send_builder = "<";
  va_list ids_to_send;
  va_start(ids_to_send, nbs);

  for (int i = 0; i < nbs; ++i)
  {
    int id = va_arg ( ids_to_send, int );
    uint8_t buffer_out[MAX_MSG_LEN];
    char to_send[10];  // The biggest byte array in the nanopb encode seems to be 10 (should be coded to not depend on it tho...)
    pb_ostream_t stream = pb_ostream_from_buffer(buffer_out, sizeof(buffer_out));
    success = success && pb_encode(&stream, _id_2_type[id], _id_2_msg[id]);

    if(success)
    {
      to_send_builder += String(id);
      to_send_builder += "|";
      
      for(int j = 0; j < stream.bytes_written; j++)
      {
        sprintf (to_send, "%02X", buffer_out[j]);
        to_send_builder += String(to_send);
      }
    }
    else
      break;
    to_send_builder += ";";
  }
  to_send_builder += ">";
  va_end(ids_to_send);

  if (success)
    Serial.print(to_send_builder);
  
  return success;
}

/*
 * Convert an input string to a list of PB messages and ids
 * 
 * @param in_string: The input string to parse
 * @param msg_ids: Output a list of id of all the sub messages in the full message
 * @param out_strings: Output a list of str containting the messages (without id)
 * 
 * @return: The number of submessages
 */
int PBUtils::parse_msg(char in_string[], int *msg_ids, char **out_strings)
{
  // Split all messages
  char *whole_msgs[MAX_MSG_LEN];
  int ind = 0;
  char* substr = strtok(in_string, ";");

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
    out_strings[i] = msg;
  }

  return ind;
}

/*
 * Convert a string to a list of uint
 *
 * @param in_string: The input string to convert to hex
 * @param string_values: The output of the conversion (size divided by 2 since 2 HEX is on uint8_t)
 * 
 * @return: 
 */
void PBUtils::chars2bytes(char* in_string, uint8_t* string_value)
{
  // Jai pt fucker dequoi, a retester ...
  int len = strlen(in_string)/2;
  for (int i = 0; i < len; ++i)
    string_value[i] = char2hex(in_string[(i*2)], in_string[(i*2)+1]);
}

/*
 * Convert two char (between 00 and FF) to an HEX value 
 * 
 * @param in: Array of two char to be converted
 * 
 * @return: The decimal value
 */
uint8_t PBUtils::char2hex(char in1, char in2)
{
  uint8_t val[2];
  char in[2] = { in1, in2};

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
