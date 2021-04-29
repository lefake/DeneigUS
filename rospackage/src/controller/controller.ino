#include <pb_common.h>
#include <pb.h>
#include <pb_encode.h>
#include <pb_decode.h>

#include "PBUtils.h"
#include "twist.pb.h"
#include "floatarray.pb.h"
#include "range.pb.h"

#include "Sonars.h"
#include "Motor.h"
#include "IMU.h"
#include "Gps.h"
#include "constants.h"
#include "pins.h"

// Functions 
void cmd_vel_callback();
void cmd_tourelle_callback();

// GLOBALS
long last_time = 0;
long delay_interval = 10;

long last_time_sonar = 0;
long delay_interval_sonar = 100;

FloatArray debug_arduino_msg = FloatArray_init_zero;
Twist cmd_vel_msg = Twist_init_zero;
Twist cmd_tourelle_msg = Twist_init_zero;
Twist pos_msg = Twist_init_zero;
Range obs_pos_msg = Range_init_zero;
FloatArray imu_data_msg = FloatArray_init_zero;

Topic topics[] = {
      {DEBUG_ARDUINO, FloatArray_fields, &debug_arduino_msg},
      {CMD_VEL, Twist_fields, &cmd_vel_msg},
      {CMD_TOURELLE, Twist_fields, &cmd_tourelle_msg},
      {POS, Twist_fields, &pos_msg},
      {OBS_POS, Range_fields, &obs_pos_msg},
      {IMU_DATA, FloatArray_fields, &imu_data_msg},
      /*{ESTOP_STATE, FloatArray_fields, NULL},
      {TELE_BATT, FloatArray_fields, NULL},
      {POS_TOURELLE, FloatArray_fields, NULL},
      {DEBUG_MOT, FloatArray_fields, NULL},
      {GPS_DATA, FloatArray_fields, NULL},
      {IMU_DATA, FloatArray_fields, NULL},*/
    };

// PB Communication
int ind = 0;
char in_char;
boolean recv_in_progress = false;
bool in_cmd_complete = false;
int nbs_new_msgs = 0;
int new_msgs_ids[MAX_NBS_MSG];
char in_cmd[MAX_MSG_LEN];
PBUtils pbutils(topics, sizeof(topics) / sizeof(Topic));

// Motors
Motor m_l, m_r;
bool has_motor = true;
float vel_left = 0;
float vel_right = 0;

// Sonars
Sonars sonars;
bool has_sonars = true;
int sonars_msg_seq = 0;

// IMU
IMU imu;
bool has_imu = false;


// GPS
Gps gps;
bool has_gps = false;

void setup()
{
  debug_arduino_msg.data_count = 2;
  
  if(has_sonars)
  {
    sonars.init(sonars_trigger_pin, sonars_echo_pins);
  }
  
  if (has_motor)
  {
    m_l.init(forw_left, back_left, pwm_left);
    m_r.init(forw_right, back_right, pwm_right);
  }

  if (has_imu)
  {
    imu.init();
    imu.calibrateGyroAcc();
    imu_data_msg.data_count = IMU_DATA_MSG_ARRAY_LEN;
  }

  if (has_gps)
    gps.init();
  
  Serial.begin(115200);
}

void loop()
{
  if (millis() - last_time > delay_interval)
  {
    // To create the Map TF in tf_broadcaster
    pbutils.pb_send(1, POS);
    
    // send imu
    if (has_imu)
    {
      imu.getValuesRos(&imu_data_msg, delay_interval);
      pbutils.pb_send(1, IMU_DATA);
    }
    
    if (in_cmd_complete)
    {
      bool status = pbutils.decode_pb(in_cmd, new_msgs_ids, nbs_new_msgs);
      
      if (status && nbs_new_msgs > 0)
      {
        for (int i = 0; i < nbs_new_msgs; ++i)
        {
          switch (new_msgs_ids[i])
          {
            case CMD_VEL:
              if (has_motor)
              {
                cmd_vel_callback();
                m_l.set_speed(vel_left);
                m_r.set_speed(vel_right);
              }
              break;
              
            case CMD_TOURELLE:
              cmd_tourelle_callback();
              break;
              
            default:
              // Not a subscriber topic
              break;
          }
        }
      }
      in_cmd_complete = false;
      last_time = millis();
    }
  }
  if (millis() - last_time_sonar > delay_interval_sonar)
  {
    last_time_sonar = millis();
    if(has_sonars)
    {
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
  }
}


// CALLBACKS

void cmd_vel_callback ()
{
  vel_left = cmd_vel_msg.lx;
  vel_right = cmd_vel_msg.ly;
}

void cmd_tourelle_callback ()
{

}

void serialEvent()
{
   while (Serial.available() > 0) 
   {
      in_char = Serial.read();
      if (recv_in_progress) 
      {
         if (in_char != '>') 
         {
            in_cmd[ind++] = in_char;
            
            if (ind > MAX_MSG_LEN)    // Do better ?
              ind = MAX_MSG_LEN;
         }
         else 
         {
             in_cmd[ind] = '\0'; // terminate the string
             recv_in_progress = false;
             ind = 0;
             in_cmd_complete = true;
         }
      }
      else if (in_char == '<') 
        recv_in_progress = true;
   }
}
