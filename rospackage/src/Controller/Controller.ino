// ======================================== INCLUDES ========================================
#include <pb_common.h>
#include <pb.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include "floatarray.pb.h"
#include "int32.pb.h"

#include "Configuration.h"
#include "Constants.h"
#include "Acknowledge.h"
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
#include "MPU.h"
#endif

#ifdef HAS_GPS
#include "Gps.h"
#endif

#ifdef HAS_ENCODERS
#include "Encoder.h"
#endif

#ifdef HAS_SERVOS
#include "Servos.h"
#endif

// ======================================== FUNCTIONS ========================================
void propCallback();
void chuteCallback();
void soufflanteCmdCallback();
void deadmanCallback();

void loopSonars();
void loopController();

// ======================================== VARIABLES ========================================

// ==================== TIMERS ====================
long lastTime = 0;
long delayInterval = 100;

long lastTimeSonar = 0;
long delayIntervalSonar = 1500;

long lastTimeImu = 0;
long delayIntervalImu = 10;

// ==================== TOPICS ====================
// Out
FloatArray debugArduinoMsg = FloatArray_init_zero;
FloatArray encMsg = FloatArray_init_zero;
FloatArray imuMsg = FloatArray_init_zero;
FloatArray gpsMsg = FloatArray_init_zero;
FloatArray sonarPairsMsg = FloatArray_init_zero;
Int32 soufflanteHeightMsg = Int32_init_zero;
Int32 estopMsg = Int32_init_zero;

// In
FloatArray propMsg = FloatArray_init_zero;
FloatArray chuteMsg = FloatArray_init_zero;
Int32 soufflanteCmdMsg = Int32_init_zero;
Int32 deadmanMsg = Int32_init_zero;

const Topic topics[] = {
      // Out
      {DEBUG_ARDUINO, FloatArray_fields, &debugArduinoMsg},
      {ENC, FloatArray_fields, &encMsg},
      {IMU, FloatArray_fields, &imuMsg},
      {GPS, FloatArray_fields, &gpsMsg},
      {SONAR_PAIRS, FloatArray_fields, &sonarPairsMsg},
      {SOUFFLANTE_HEIGHT, Int32_fields, &soufflanteHeightMsg},      
      {ESTOP_STATE, Int32_fields, &estopMsg},

      // In
      {PROP, FloatArray_fields, &propMsg},
      {CHUTE, FloatArray_fields, &chuteMsg},
      {SOUFFLANTE_CMD, Int32_fields, &soufflanteCmdMsg},
      {DEADMAN, Int32_fields, &deadmanMsg},
    };

// ==================== SERIAL COMMUNICATION ====================
const String START_DELIMITER = "<{";
const String END_DELIMITER = ">}";

bool recvInProgress = false;
int inCmdIndex = 0;
char inCmd[MAX_MSG_LEN] = { "\0" };
bool inCmdComplete = false;
int inCmdType = -1;
int nbsNewMsgs = 0;
int newMsgsIds[MAX_NBS_MSG];
bool msgDiscardedLength = false;

PBUtils pbUtils(topics);
AckHandler ackHandler;

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
MPU imu;
#endif

#ifdef HAS_GPS
Gps gps;
#endif

#ifdef HAS_ENCODERS
Encoder encoders;
#endif

#ifdef HAS_SERVOS
Servos servos;
#endif

// ======================================== MAIN ========================================

void setup()
{
  Serial.begin(115200);

#ifdef CONFIGURATION_MODE
  ackHandler.writeIdToEEPROM(); // Left empty to force compile error and make sure the right id is writen
#else
  ackHandler.readIdFromEEPROM();
#endif
  
#ifdef HAS_SONARS
  sonars.init(sonarsTriggerPin, sonarsEchoPins);
#endif

#ifdef HAS_MOTOR_PROP
  motorLeft.init(motorForwardLeft, motorPwmLeft);
  motorRight.init(motorForwardRight, motorPwmRight);
#endif

#ifdef HAS_IMU
  imu.init();
#ifdef CONFIGURATION_MODE
  imu.doCalibration();
#else
  imu.loadCalibration();
#endif
#endif

#ifdef HAS_GPS
  gps.init();
#endif

#ifdef HAS_ENCODERS
  encoders.init(encoderCSPins);
#endif

#ifdef HAS_SERVOS
  servos.init(servoPins);
#endif
}

void loop()
{
  if (inCmdComplete && inCmdType == STATUS_MSGS)
    inCmdComplete = !ackHandler.acknowldgeArduino(inCmd);
  
#ifndef CONFIGURATION_MODE
  if (ackHandler.getId() == CONTROLLER)
    loopController();
  else if (ackHandler.getId() == SENSORS)
    loopSonars();
  else if (ackHandler.getId() == BATTERY)
    sendStatusWithMessage(ERROR, OTHER, "No loop for the battery arduino yet");
  else
    sendStatusWithMessage(FATAL, OTHER, "Arduino ID not valid");
#endif

  // Send status if any errors
  if(msgDiscardedLength)
  {
    sendStatus(ERROR, SERIAL_COMMUNICATION);
    msgDiscardedLength = false;
  }
}

// ======================================== CALLBACKS ========================================

void propCallback()
{
#ifdef HAS_MOTOR_PROP
  motorLeft.setSpeed(propMsg.data[0]);
  motorRight.setSpeed(propMsg.data[1]);
#endif
}

void chuteCallback()
{
#ifdef HAS_SERVOS
  servos.setPos(ROTATION, chuteMsg.data[0]);
  servos.setPos(ELEVATION, chuteMsg.data[1]);
#endif

#ifdef HAS_MOTOR_BLOW
  // TODO : Set motor speed
#endif
}

void soufflanteCmdCallback()
{
  // TODO
}

void deadmanCallback()
{
  // TODO
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
        inCmdType = endDelimIndex;
        inCmdComplete = true;
      }
    }
    else if (startDelimIndex != -1)
      recvInProgress = true;
  }
}



// ======================================== LOOPS ========================================

// CONTROLLER
void loopController()
{
#ifdef HAS_IMU
  long period = millis() - lastTimeImu;
  if (period > delayIntervalImu)
  {
    if (period > 50)
    {
      sendStatusWithMessage(FATAL, IMU_DEVICE, "Frequency was not met");
    }
    else
    {
      imu.getValues(&imuMsg);
      pbUtils.pbSend(1, IMU);
    }
    lastTimeImu = millis();
  }
#endif
  
  if (millis() - lastTime > delayInterval)
  {
    lastTime = millis();
#ifdef HAS_ENCODERS
    float leftSpeed = encoders.getEncVel(0, delayInterval);
    float rightSpeed = encoders.getEncVel(1, delayInterval);
    encMsg.data[0] = (leftSpeed + rightSpeed) / 2;
    encMsg.data[1] = (leftSpeed - rightSpeed) / TRACK_WIDTH;
    pbUtils.pbSend(1, ENC);
#endif

#ifdef HAS_GPS
    gps.getCoordinates(&gpsMsg);
    pbUtils.pbSend(1, GPS);
#endif

#ifdef DEBUGGING
    encMsg.data_count = 2;
    encMsg.data[0] = 1;
    encMsg.data[1] = 3;
    pbUtils.pbSend(1, ENC);
#endif

    if (inCmdComplete)
    {
      inCmdComplete = false;
      bool status = pbUtils.decodePb(inCmd, newMsgsIds, nbsNewMsgs);
      
      if (status && nbsNewMsgs > 0)
      {
        for (int i = 0; i < nbsNewMsgs; ++i)
        {
          switch (newMsgsIds[i])
          {
            case PROP:
              propCallback();
              break;
              
            case CHUTE:
              chuteCallback();
              break;

            case SOUFFLANTE_CMD:
              soufflanteCmdCallback();
              break;

            case DEADMAN:
              deadmanCallback();
              break;
              
            default:
              sendStatusWithMessage(WARNING, OTHER, "Unsupported topic:" + String(newMsgsIds[i]));
              break;
          }
        }
      }
      else
        sendStatus(ERROR, DECODING_PB);
      inCmdType = -1;
    }
  }
}

// SONARS
void loopSonars()
{
  if (millis() - lastTimeSonar > delayIntervalSonar)
  {
    lastTimeSonar = millis();

#ifdef HAS_SONARS
    for (int i = 0; i < NBS_SONARS/2; ++i)
    {
      sonars.readPair(i, &sonarPairsMsg);
      pbUtils.pbSend(1, SONAR_PAIRS);
    }
#endif
    
    if (inCmdComplete)
    {
      inCmdComplete = false;
      bool status = pbUtils.decodePb(inCmd, newMsgsIds, nbsNewMsgs);
      
      if (status && nbsNewMsgs > 0)
      {
        for (int i = 0; i < nbsNewMsgs; ++i)
        {
          switch (newMsgsIds[i])
          {
            case PROP:
            case CHUTE:
            case SOUFFLANTE_HEIGHT:
              break;
  
            case DEADMAN:
              deadmanCallback();
              break;
              
            default:
              sendStatusWithMessage(WARNING, OTHER, "Unsupported topic:" + String(newMsgsIds[i]));
              break;
          }
        }
      }
      else
        sendStatus(ERROR, DECODING_PB);
      inCmdType = -1;
    }
  }
  

}
