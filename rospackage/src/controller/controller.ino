// ======================================== INCLUDES ========================================
#include <pb_common.h>
#include <pb.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include "Configuration.h"
#include "constants.h"
#include "StatusMessage.h"
#include "PBUtils.h"
#include "twist.pb.h"
#include "floatarray.pb.h"
#include "range.pb.h"
#include "pins.h"

// ======================================== CONDITIONNAL INCLUDES ========================================

#ifdef HAS_SONARS
#include "Sonars.h"
#endif

#ifdef HAS_MOTOR_PROP
#include "Motor.h"
#endif

#ifdef HAS_IMU
#include "IMU.h"
#endif

#ifdef HAS_GPS
#include "Gps.h"
#endif

// ======================================== FUNCTIONS ========================================
void cmd_vel_callback();
void cmd_tourelle_callback();
void broadcastId();
void acknowldgeArduino();
bool parseAcknowledgeMessage(char* msg);

// ======================================== VARIABLES ========================================

// ==================== TIMERS ====================
int val = 0;
long last_time = 0;
long delay_interval = 50;

long last_time_sonar = 0;
long delay_interval_sonar = 250;

// ==================== TOPICS ====================
FloatArray debug_arduino_msg = FloatArray_init_zero;
Twist cmd_vel_msg = Twist_init_zero;
Twist cmd_tourelle_msg = Twist_init_zero;
Twist pos_msg = Twist_init_zero;
Range obs_pos_msg = Range_init_zero;
Twist imu_data_msg = Twist_init_zero;

const Topic topics[] = {
      {DEBUG_ARDUINO, FloatArray_fields, &debug_arduino_msg},
      {CMD_VEL, Twist_fields, &cmd_vel_msg},
      {CMD_TOURELLE, Twist_fields, &cmd_tourelle_msg},
      {POS, Twist_fields, &pos_msg},
      {OBS_POS, Range_fields, &obs_pos_msg},
      {IMU_DATA, Twist_fields, &imu_data_msg},
    };

// ==================== SERIAL COMMUNICATION ====================
const String START_DELIMITER = "<{";
const String END_DELIMITER = ">}";
const String ARDUINO_ID = "SENSORS"; // Put in EEPROM ?

bool recv_in_progress = false;
int ind = 0;
char in_cmd[MAX_MSG_LEN] = { "\0" };
int in_cmd_complete = -1;
int nbs_new_msgs = 0;
int new_msgs_ids[MAX_NBS_MSG];
PBUtils pbutils(topics, sizeof(topics) / sizeof(Topic));

bool ack_recieved = false;
bool msgDiscardedLength = false;

// ==================== DEVICES ====================
#ifdef HAS_MOTOR_PROP
Motor m_l, m_r;
float vel_left = 0;
float vel_right = 0;
#endif

#ifdef HAS_SONARS
Sonars sonars;
int sonars_msg_seq = 0;
#endif

#ifdef HAS_IMU
IMU imu;
#endif

#ifdef HAS_GPS
Gps gps;
#endif

// ======================================== MAIN ========================================

void setup()
{
  Serial.begin(115200);
  
#ifdef HAS_SONARS
  sonars.init(sonars_trigger_pin, sonars_echo_pins);
#endif
  
#ifdef HAS_MOTOR_PROP
    m_l.init(forw_left, back_left, pwm_left);
    m_r.init(forw_right, back_right, pwm_right);
#endif

#ifdef HAS_HAS_IMU
  imu.init();
#endif

#ifdef HAS_GPS
  gps.init();
#endif

  //debug_arduino_msg.data_count = 2;
}

void loop()
{
  if (ack_recieved)
  {
    if (millis() - last_time > delay_interval)
    {
#ifndef HAS_IMU
#ifdef DEBUGGING
      // To create the Map TF in tf_broadcaster
      // This is a patch in the case there's no IMU/Encoder connected
      pbutils.pb_send(1, POS);
#endif
#endif

#ifdef HAS_IMU
      imu.getValues(&imu_data_msg, delay_interval);
      pbutils.pb_send(1, IMU_DATA);
#endif

      // Command received
      switch (in_cmd_complete)
      {
        case DATA_MSGS:
          bool status = pbutils.decode_pb(in_cmd, new_msgs_ids, nbs_new_msgs);
          
          if (status && nbs_new_msgs > 0)
          {
            for (int i = 0; i < nbs_new_msgs; ++i)
            {
              switch (new_msgs_ids[i])
              {
                case CMD_VEL:
#ifdef HAS_MOTOR_PROP
                  cmd_vel_callback();
                  m_l.set_speed(vel_left);
                  m_r.set_speed(vel_right);
#endif
                  break;
                  
                case CMD_TOURELLE:
                  cmd_tourelle_callback();
                  break;
                  
                default:
                  sendStatusWithMessage(WARNING, OTHER, "Unsupported topic:" + String(new_msgs_ids[i]));
                  break;
              }
            }
          }
          else
          {
            sendStatus(ERROR, DECODING_PB);
          }
          in_cmd_complete = -1;
          break;

        case META_MSGS:
          in_cmd_complete = -1;
          break;

        default:
          sendStatusWithMessage(WARNING, SERIAL_COMMUNICATION, "Unknown message type" + String(in_cmd_complete));
          in_cmd_complete = -1;
          break;
      }
      last_time = millis();
    }


// ======================================== SONARS LOOP ========================================
// Will be move to another arduino
#ifdef HAS_SONARS
    if (millis() - last_time_sonar > delay_interval_sonar)
    {
      last_time_sonar = millis();
      char frame_id[50] = "";
      for (int i = 0; i < NBS_SONARS; ++i)
      {
        (String("sonar_f_") + String(i)).toCharArray(frame_id, sizeof(frame_id));
        obs_pos_msg.range = sonars.dist(i);
        obs_pos_msg.seq = sonars_msg_seq++;
        memcpy(obs_pos_msg.frame_id, frame_id, sizeof(frame_id)/sizeof(frame_id[0]));
        pbutils.pb_send(1, OBS_POS);
      }
    }
#endif
  }
  else
    acknowldgeArduino();
  
  // Send status if any errors
  if(msgDiscardedLength)
  {
    sendStatus(ERROR, SERIAL_COMMUNICATION);
    msgDiscardedLength = false;
  }
}

// ======================================== CALLBACKS ========================================

void cmd_vel_callback ()
{
#ifdef HAS_MOTOR_PROP
  vel_left = cmd_vel_msg.lx;
  vel_right = cmd_vel_msg.ly;
#endif
}

void cmd_tourelle_callback ()
{

}

// ======================================== ACKNOWLEDGE ========================================

void acknowldgeArduino()
{
  if (in_cmd_complete == META_MSGS)
  {
    if (parseAcknowledgeMessage(in_cmd))
      ack_recieved = true;
      
    in_cmd_complete = -1;
  }

  broadcastId();
  delay(ID_BROADCAST_DELAY);
}

bool parseAcknowledgeMessage(char* msg)
{
  char* type = strtok(msg, ";");
  char* id = strtok(NULL, ";");
  if (String(type).toInt() == ACKNOWLEDGE && String(id).equals(ARDUINO_ID))
    return true;

  return false;
}

void broadcastId()
{
  sendStatusWithMessage(-1, ACKNOWLEDGE, ARDUINO_ID);
}

// ======================================== SERIAL ========================================
void serialEvent()
{
  while (Serial.available() > 0)
  {
    char in_char = Serial.read();
    int start_del_ind = START_DELIMITER.indexOf(in_char);

    if (recv_in_progress)
    {
      int end_del_ind = END_DELIMITER.indexOf(in_char); // If start and end index is diff, will always return end
      if (end_del_ind == -1)
      {
        in_cmd[ind++] = in_char;

        if (ind >= MAX_MSG_LEN-1) // If the last char isn't the end delimiter the message is not going to fit
        {
          msgDiscardedLength = true;
          in_cmd[ind] = '\0';
          recv_in_progress = false;
          ind = 0;
        }
      }
      else
      {
        in_cmd[ind] = '\0';
        recv_in_progress = false;
        ind = 0;
        in_cmd_complete = end_del_ind;
      }
    }
    else if (start_del_ind != -1)
      recv_in_progress = true;
  }
}
