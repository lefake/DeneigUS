#include <pb_common.h>
#include <pb.h>
#include <pb_encode.h>
#include <pb_decode.h>

#include "msg/deneigus/twist.pb.h"
#include "msg/deneigus/floatarray.pb.h"
#include "PBUtils.h"
#include "Motor.h"
#include "pins.h"

// Communication
bool inCmdComplete = false;
char inCmd[MAX_MSG_LEN];

// PB
int nbs_new_msgs = 0;
int new_msgs_ids[MAX_NBS_MSG];
PBUtils pbutils;

FloatArray debug_arduino_msg = FloatArray_init_zero;
Twist cmd_vel_msg = Twist_init_zero;
Twist cmd_tourelle_msg = Twist_init_zero;

// IMPORTANT : SAME ORDER AS THE ENUM IN PBConstants.h OTHERWISE IT'LL FAIL WHILE DECODING PB
void *msgs[_NBS_TOPICS] = { &debug_arduino_msg, &cmd_vel_msg, &cmd_tourelle_msg };


// Timing 
long last = 0;
long inter = 10;

int value = 0;

Motor m_l, m_r;
float vel_left = 0;
float vel_right = 0;

void setup() {
  Serial.begin(115200);

  m_l.init(forw_left, back_left, pwm_left);
  m_r.init(forw_right, back_right, pwm_right);
}

void loop() {
  if (millis() - last > inter)
  {
    last = millis();

    m_l.set_speed(vel_left);
    m_r.set_speed(vel_right);

    if (inCmdComplete)
    {
      bool status = pbutils.decode_pb(inCmd, msgs, new_msgs_ids, nbs_new_msgs);

      if (status && nbs_new_msgs > 0)
      {
        for (int i = 0; i < nbs_new_msgs; ++i)
        {
          switch (new_msgs_ids[i])
          {
            case CMD_VEL:
              float lin = cmd_vel_msg.lx;
              float rot = cmd_vel_msg.az;
              
              vel_left = CALC(lin, -rot);
              vel_right = CALC(lin, rot);
              /*debug_arduino_msg.data_count = 6;
              debug_arduino_msg.data[0] = cmd_vel_msg.lx;
              debug_arduino_msg.data[1] = cmd_vel_msg.ly;
              debug_arduino_msg.data[2] = cmd_vel_msg.lz;
              debug_arduino_msg.data[3] = cmd_vel_msg.ax;
              debug_arduino_msg.data[4] = cmd_vel_msg.ay;
              debug_arduino_msg.data[5] = cmd_vel_msg.az;    

              int bob[] = { 0 };
              pbutils.pb_send(bob, msgs);*/
              break;
              
            case CMD_TOURELLE:
              /*debug_arduino_msg.data_count = 6;
              debug_arduino_msg.data[0] = cmd_vel_msg.lx + 10;
              debug_arduino_msg.data[1] = cmd_vel_msg.ly + 10;
              debug_arduino_msg.data[2] = cmd_vel_msg.lz + 10;
              debug_arduino_msg.data[3] = cmd_vel_msg.ax + 10;
              debug_arduino_msg.data[4] = cmd_vel_msg.ay + 10;
              debug_arduino_msg.data[5] = cmd_vel_msg.az + 10;*/
              break;
              
            default:
              // Not a subscriber topic
              break;
          }
        }
      }
      inCmdComplete = false;
    }
  }
}

void serialEvent()
{
   static boolean recvInProgress = false;
   static int ind = 0;
   char inChar;

   while (Serial.available() > 0) 
   {
      inChar = Serial.read();
      if (recvInProgress) 
      {
         if (inChar != '>') 
         {
            inCmd[ind++] = inChar;
            
            if (ind > MAX_MSG_LEN)    // Do better ?
              ind = MAX_MSG_LEN;
         }
         else 
         {
             inCmd[ind] = '\0'; // terminate the string
             recvInProgress = false;
             ind = 0;
             inCmdComplete = true;
         }
      }
      else if (inChar == '<') 
        recvInProgress = true;
   }
}
