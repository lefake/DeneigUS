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
  for (int i = 0; i < nbs_topics; ++i)
  {
    _id_2_type[topics[i].id] = topics[i].type;
    _id_2_msg[topics[i].id] = topics[i].msg;
  }
  
  for (int i = 0; i < 10; ++i)
    dict[i] = i;

  for (int i = 'a'; i <= 'f'; ++i)
    dict[i-'0'] = i - 'a' + 10;
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
 * @param ids: The list of ids to send in the same messages
 *             The id list has to be initialized (ex : cant use it like this pb_send({ 0, 1 });
 * 
 * @return: If the encode was successful  
 */
bool PBUtils::pb_send(int* ids)
{
  bool success = true;
  int nbs_to_send = sizeof(ids)/sizeof(ids[0]);
  String to_send_builder = "<";

  for (int i = 0; i < nbs_to_send; ++i)
  {
    uint8_t buffer_out[MAX_MSG_LEN];
    char to_send[2];
    pb_ostream_t stream = pb_ostream_from_buffer(buffer_out, sizeof(buffer_out));
    success = success && pb_encode(&stream, _id_2_type[ids[i]], _id_2_msg[ids[i]]);

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
  int len = strlen(in_string)/2;
  for (int i = 0; i < len; ++i)
  {
    string_value[i] =  dict[in_string[(i*2)] - '0']*16 + dict[in_string[(i*2)+1] - '0'];
  }
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

  //for (int i = 0; i < 2; i++)
  {
    /*if (in[i] >= 'A' && in[i] <= 'F')
      val[i] = in[i] - 'A' + 10;
    else */
    
    /*if (in[i] >= 'a' && in[i] <= 'f')
      val[i] = in[i] - 'a' + 10;
    else
      val[i] = in[i] - '0';*/
  }
  val['0' - '0']=1;
 
  return val[0]*16 + val[1];
}
