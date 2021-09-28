// ======================================== INCLUDES ========================================
#include <pb_common.h>
#include <pb.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include "twist.pb.h"
#include "floatarray.pb.h"
#include "range.pb.h"

#include "Configuration.h"
#include "Constants.h"
#include "StatusMessage.h"
#include "PBUtils.h"
#include "Pins.h"


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
void cmdVelCallback();
void cmdTourelleCallback();
void broadcastId();
void acknowldgeArduino();
bool parseAcknowledgeMessage(char* msg);

// ======================================== VARIABLES ========================================

// ==================== TIMERS ====================
int val = 0;
long lastTime = 0;
long delayInterval = 1000;

long lastTimeSonar = 0;
long delayIntervalSonar = 250;

// ==================== TOPICS ====================
FloatArray debugArduinoMsg = FloatArray_init_zero;
Twist cmdVelMsg = Twist_init_zero;
Twist cmdTourelleMsg = Twist_init_zero;
Twist posMsg = Twist_init_zero;
Range obsPosMsg = Range_init_zero;
Twist imuDataMsg = Twist_init_zero;

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! DONT USE THE POS TOPIC IT'S BUGGED !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! TODO : FIX IT DUS-499              !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

const Topic topics[] = {
      {DEBUG_ARDUINO, FloatArray_fields, &debugArduinoMsg},
      {CMD_VEL, Twist_fields, &cmdVelMsg},
      {CMD_TOURELLE, Twist_fields, &cmdTourelleMsg},
      {POS, Twist_fields, &posMsg},
      {OBS_POS, Range_fields, &obsPosMsg},
      {IMU_DATA, Twist_fields, &imuDataMsg},
    };

// ==================== SERIAL COMMUNICATION ====================
const String START_DELIMITER = "<{";
const String END_DELIMITER = ">}";
const String ARDUINO_ID = "SENSORS"; // Put in EEPROM ?

bool recvInProgress = false;
int inCmdIndex = 0;
char inCmd[MAX_MSG_LEN] = { "\0" };
int inCmdComplete = -1;
int nbsNewMsgs = 0;
int newMsgsIds[MAX_NBS_MSG];
PBUtils pbUtils(topics, sizeof(topics) / sizeof(Topic));

bool ackRecieved = false;
bool msgDiscardedLength = false;

// ==================== DEVICES ====================
#ifdef HAS_MOTOR_PROP
Motor motorLeft, motorRight;
float motorVelLeft = 0;
float motorVelRight = 0;
#endif

#ifdef HAS_SONARS
Sonars sonars;
int sonarsMsgSeq = 0;
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
  sonars.init(sonarsTriggerPin, sonarsEchoPins);
#endif
  
#ifdef HAS_MOTOR_PROP
    motorLeft.init(motorForwardLeft, motorBackwardLeft, motorPwmLeft);
    motorRight.init(motorForwardRight, motorBackwardRight, motorPwmRight);
#endif

#ifdef HAS_HAS_IMU
  imu.init();
#endif

#ifdef HAS_GPS
  gps.init();
#endif
}

void loop()
{
  if (ackRecieved)
  {
    if (millis() - lastTime > delayInterval)
    {
#ifdef DEBUGGING
      // To create the Map TF in tf_broadcaster
      // This is a patch in the case there's no IMU/Encoder connected
      pbUtils.pbSend(1, POS);
#endif

#ifdef HAS_IMU
      imu.getValues(&imuDataMsg, delayInterval);
      pbUtils.pbSend(1, IMU_DATA);
#endif

      // Command received
      switch (inCmdComplete)
      {
        case DATA_MSGS:
          bool status = pbUtils.decodePb(inCmd, newMsgsIds, nbsNewMsgs);
          
          if (status && nbsNewMsgs > 0)
          {
            for (int i = 0; i < nbsNewMsgs; ++i)
            {
              switch (newMsgsIds[i])
              {
                case CMD_VEL:
#ifdef HAS_MOTOR_PROP
                  cmdVelCallback();
                  motorLeft.setSpeed(motorVelLeft);
                  motorRight.setSpeed(motorVelRight);
#endif
                  break;
                  
                case CMD_TOURELLE:
                  cmdTourelleCallback();
                  break;
                  
                default:
                  sendStatusWithMessage(WARNING, OTHER, "Unsupported topic:" + String(newMsgsIds[i]));
                  break;
              }
            }
          }
          else
          {
            sendStatus(ERROR, DECODING_PB);
          }
          inCmdComplete = -1;
          break;

        case META_MSGS:
          inCmdComplete = -1;
          break;

        default:
          sendStatusWithMessage(WARNING, SERIAL_COMMUNICATION, "Unknown message type" + String(inCmdComplete));
          inCmdComplete = -1;
          break;
      }
      lastTime = millis();
    }


// ======================================== SONARS LOOP ========================================
// Will be move to another arduino
#ifdef HAS_SONARS
    if (millis() - lastTimeSonar > delayIntervalSonar)
    {
      lastTimeSonar = millis();
      char frameId[50] = "";
      for (int i = 0; i < NBS_SONARS; ++i)
      {
        (String("sonar_f_") + String(i)).toCharArray(frameId, sizeof(frameId));
        obsPosMsg.range = sonars.dist(i);
        obsPosMsg.seq = sonars_msg_seq++;
        memcpy(obsPosMsg.frame_id, frameId, sizeof(frameId)/sizeof(frameId[0]));
        pbUtils.pbSend(1, OBS_POS);
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

void cmdVelCallback ()
{
#ifdef HAS_MOTOR_PROP
  motorVelLeft = cmdVelMsg.lx;
  motorVelRight = cmdVelMsg.ly;
#endif
}

void cmdTourelleCallback ()
{

}

// ======================================== ACKNOWLEDGE ========================================

void acknowldgeArduino()
{
  if (inCmdComplete == META_MSGS)
  {
    if (parseAcknowledgeMessage(inCmd))
      ackRecieved = true;
      
    inCmdComplete = -1;
  }

  sendStatusWithMessage(NONE, ID, ARDUINO_ID);
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

// ======================================== SERIAL ========================================
void serialEvent()
{
  while (Serial.available() > 0)
  {
    char inChar = Serial.read();
    int startDelimIndex = START_DELIMITER.indexOf(inChar);

    if (recvInProgress)
    {
      int endDelimIndex = END_DELIMITER.indexOf(inChar); // If start and end index is diff, will always return end
      if (endDelimIndex == -1)
      {
        inCmd[inCmdIndex++] = inChar;

        if (inCmdIndex >= MAX_MSG_LEN-1) // If the last char isn't the end delimiter the message is not going to fit
        {
          msgDiscardedLength = true;
          inCmd[inCmdIndex] = '\0';
          recvInProgress = false;
          inCmdIndex = 0;
        }
      }
      else
      {
        inCmd[inCmdIndex] = '\0';
        recvInProgress = false;
        inCmdIndex = 0;
        inCmdComplete = endDelimIndex;
      }
    }
    else if (startDelimIndex != -1)
      recvInProgress = true;
  }
}
