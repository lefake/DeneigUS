// ======================================== INCLUDES ========================================
#include <pb_common.h>
#include <pb.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include "twist.pb.h"
#include "floatarray.pb.h"
#include "int32.pb.h"

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

#ifdef HAS_ENCODERS
#include "Encoder.h"
#endif

#ifdef HAS_SERVOS
#include "Servos.h"
#endif

// ======================================== FUNCTIONS ========================================
void loopSonars();
void loopController();

void propCallback();
void chuteCallback();
void soufflanteHeightCallback();
void controlModeCallback();
void deadmanCallback();

void broadcastId();
void acknowldgeArduino();
bool parseAcknowledgeMessage(char* msg);

// ======================================== VARIABLES ========================================

// ==================== TIMERS ====================
int val = 0;
long lastTime = 0;
long delayInterval = 100;

long lastTimeSonar = 0;
long delayIntervalSonar = 1500;

// ==================== TOPICS ====================
// Out
FloatArray debugArduinoMsg = FloatArray_init_zero;
FloatArray encMsg = FloatArray_init_zero;
FloatArray imuMsg = FloatArray_init_zero;
FloatArray gpsMsg = FloatArray_init_zero;
FloatArray sonarPairsMsg = FloatArray_init_zero;
Int32 estopMsg = Int32_init_zero;

// In
FloatArray propMsg = FloatArray_init_zero;
FloatArray chuteMsg = FloatArray_init_zero;
Int32 soufflanteHeightMsg = Int32_init_zero;
Int32 deadmanMsg = Int32_init_zero;

const Topic topics[] = {
      // Out
      {DEBUG_ARDUINO, FloatArray_fields, &debugArduinoMsg},
      {ENC, FloatArray_fields, &encMsg},
      {IMU, FloatArray_fields, &imuMsg},
      {GPS, FloatArray_fields, &gpsMsg},
      {SONAR_PAIRS, FloatArray_fields, &sonarPairsMsg},
      {ESTOP_STATE, Int32_fields, &estopMsg},

      // In
      {PROP, FloatArray_fields, &propMsg},
      {CHUTE, FloatArray_fields, &chuteMsg},
      {SOUFFLANTE_HEIGHT, Int32_fields, &soufflanteHeightMsg},
      {DEADMAN, Int32_fields, &deadmanMsg},
    };

// ==================== SERIAL COMMUNICATION ====================
const String START_DELIMITER = "<{";
const String END_DELIMITER = ">}";
const String ARDUINO_ID = "CONTROLLER"; // TODO : Put in EEPROM

bool recvInProgress = false;
int inCmdIndex = 0;
char inCmd[MAX_MSG_LEN] = { "\0" };
bool inCmdComplete = false;
int inCmdType = -1;
int nbsNewMsgs = 0;
int newMsgsIds[MAX_NBS_MSG];
PBUtils pbUtils(topics);

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
  
#ifdef HAS_SONARS
  sonars.init(sonarsTriggerPin, sonarsEchoPins);
#endif

#ifdef HAS_MOTOR_PROP
  motorLeft.init(motorForwardLeft, motorPwmLeft);
  motorRight.init(motorForwardRight, motorPwmRight);
#endif

#ifdef HAS_IMU
  imu.init();
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
    inCmdComplete = !acknowldgeArduino(inCmd);
  else
  {
  }

  delay(250);
  

/*
  if (ARDUINO_ID == "CONTROLLER")
    loopController();
  else
    loopSonars();
  
  // Send status if any errors
  if(msgDiscardedLength)
  {
    sendStatus(ERROR, SERIAL_COMMUNICATION);
    msgDiscardedLength = false;
  }
*/
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

void soufflanteHeightCallback()
{
  // TODO
}

void deadmanCallback()
{
  // TODO
}

// ======================================== ACKNOWLEDGE ========================================
bool acknowldgeArduino(char* msg)
{
  if (String(msg).toInt() == ACK_REQUEST_ID)
  {
    Serial.print("{");
    Serial.print(ARDUINO_ID);
    Serial.print("}");
    return true;
  }
  else
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
  if (millis() - lastTime > delayInterval)
  {
    lastTime = millis();
#ifdef HAS_ENCODERS
    encMsg.data[0] = encoders.getEncVel(0, delayInterval);
    encMsg.data[1] = encoders.getEncVel(1, delayInterval);
    pbUtils.pbSend(1, ENC);
#endif

#ifdef HAS_IMU
    imu.getValues(&imuMsg);
    pbUtils.pbSend(1, IMU);
#endif

#ifdef HAS_GPS
    gps.getCoordinates(&gpsMsg);
    pbUtils.pbSend(1, GPS);
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

            case SOUFFLANTE_HEIGHT:
              soufflanteHeightCallback();
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
  
#ifdef HAS_SONARS
  if (millis() - lastTimeSonar > delayIntervalSonar)
  {
    lastTimeSonar = millis();
    for (int i = 0; i < NBS_SONARS/2; ++i)
    {
      sonars.readPair(i, &sonarPairsMsg);
      pbUtils.pbSend(1, SONAR_PAIRS);
    }
  }
#endif
}
