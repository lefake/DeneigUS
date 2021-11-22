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

#ifdef HAS_ACTUATOR
#include "Actuator.h"
#endif

#ifdef HAS_SONARS
#include "Sonars.h"
#endif

#ifdef HAS_LIGHTTOWER
#include "LightTower.h"
#endif

// ======================================== FUNCTIONS ========================================
void propCallback();
void chuteCallback();
void soufflanteCmdCallback();
void deadmanCallback();
void estopCallback();
void lightCallback();
void pidCstCallback();

void loopSonars();
void loopSafety();
void loopController();

// ======================================== VARIABLES ========================================

// ==================== TIMERS ====================
long lastTime = 0;
long delayInterval = 100;

long lastTimeSonar = 0;
long delayIntervalSonar = 1500;

long lastTimeImu = 0;
long delayIntervalImu = 10;

long lastDebounceTime = 0;
long delayDebounceInterval = 50;

// ==================== TOPICS ====================
// Out
FloatArray debugArduinoMsg = FloatArray_init_zero;
FloatArray encMsg = FloatArray_init_zero;
FloatArray imuMsg = FloatArray_init_zero;
FloatArray gpsMsg = FloatArray_init_zero;
FloatArray sonarPairsMsg = FloatArray_init_zero;
Int32 soufflanteHeightMsg = Int32_init_zero;
Int32 estopStateMsg = Int32_init_zero;
FloatArray debugMotMsg = FloatArray_init_zero;

// In
FloatArray propMsg = FloatArray_init_zero;
FloatArray chuteMsg = FloatArray_init_zero;
Int32 soufflanteCmdMsg = Int32_init_zero;
Int32 deadmanMsg = Int32_init_zero;
Int32 estopMsg = Int32_init_zero;
FloatArray lightMsg = FloatArray_init_zero;
FloatArray pidCstMsg = FloatArray_init_zero;

const Topic topics[] = {
      // Out
      {DEBUG_ARDUINO, FloatArray_fields, &debugArduinoMsg},
      {ENC, FloatArray_fields, &encMsg},
      {IMU, FloatArray_fields, &imuMsg},
      {GPS, FloatArray_fields, &gpsMsg},
      {SONAR_PAIRS, FloatArray_fields, &sonarPairsMsg},
      {SOUFFLANTE_HEIGHT, Int32_fields, &soufflanteHeightMsg},      
      {ESTOP_STATE, Int32_fields, &estopStateMsg},
      {DEBUG_MOT, FloatArray_fields, &debugMotMsg},

      // In
      {PROP, FloatArray_fields, &propMsg},
      {CHUTE, FloatArray_fields, &chuteMsg},
      {SOUFFLANTE_CMD, Int32_fields, &soufflanteCmdMsg},
      {DEADMAN, Int32_fields, &deadmanMsg},
      {ESTOP, Int32_fields, &estopMsg},
      {LIGHT, FloatArray_fields, &lightMsg},
      {PID_CST, FloatArray_fields, &pidCstMsg},
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

// ==================== Safety=======================
bool lastEstopState = false;
bool estopState = false;
bool deadmanActive = false;

// ==================== DEVICES ====================
#ifdef HAS_MOTOR_PROP
Motor motorLeft, motorRight;
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

#ifdef HAS_ACTUATOR
Actuator actuator;
#endif
  
#ifdef HAS_SONARS
Sonars sonars;
#endif

#ifdef HAS_LIGHTTOWER
LightTower lightTower;
#endif

// ======================================== MAIN ========================================
void setup()
{
  Serial.begin(115200);

#ifdef CONFIGURATION_MODE
  ackHandler.writeIdToEEPROM(); // Left empty to force compile error and make sure the right id is writen
  while(1);
#else
  ackHandler.readIdFromEEPROM();
#endif

// ==== Controller ====

  if(ackHandler.getId() == CONTROLLER)
  {
    #ifdef HAS_MOTOR_PROP
      // Make sure the arduino is not in SPI slave mode
      pinMode(53, OUTPUT);
      digitalWrite(53,LOW);
    
      motorLeft.init(motorBackwardLeftPin, motorPwmLeftPin, csEncoderL, motorLatchLeftPin, true);
      motorRight.init(motorBackwardRightPin, motorPwmRightPin, csEncoderR, motorLatchRightPin, false);
      motorLeft.setPID(13.0, 11.0, 0);
      motorRight.setPID(42.0, 11.0, 0);
      
      motorLeft.setVoltage(0);
      motorRight.setVoltage(0);
    #endif
    
    #ifdef HAS_IMU
      imu.init();
      #ifdef CONFIGURATION_MODE
        imu.doCalibration();
      #else
        // Read actual calbration
        //imu.loadCalibration();
        // Calibrated
        imu.hardcodeCalibration();
      #endif
    #endif
    
    #ifdef HAS_GPS
      gps.init();
    #endif
    
    #ifdef HAS_SERVOS
      servos.init(servoPins);
    #endif
    
    #ifdef HAS_ACTUATOR
      actuator.init(actuatorSwitchUpPin, actuatorSwitchDownPin, actuatorUpPin, actuatorDownPin);
      soufflanteHeightMsg.data = actuator.getCurrentPos();
      pbUtils.pbSend(1, SOUFFLANTE_HEIGHT);
    #endif
  }
  
// ==== Sensors ====
  else if (ackHandler.getId() == SENSORS)
  {
    #ifdef HAS_SONARS
      sonars.init(sonarsTriggerPin, sonarsEchoPins);
    #endif
  
    #ifdef HAS_LIGHTTOWER
    lightTower.init(lightPins);
    #endif
  }
  
// ==== Safety ====
  else if (ackHandler.getId() == SAFETY)
  {
    pinMode(estopPin, OUTPUT);
    digitalWrite(estopPin, LOW);
    pinMode(estopStatePin, INPUT);
    
    estopState = digitalRead(estopStatePin);
    estopStateMsg.data = estopState;
    pbUtils.pbSend(1, ESTOP_STATE);
  }

// ==== Unknown ====
  else
  {
    sendStatusWithMessage(FATAL, OTHER, "Arduino has no valid ID");
    while(1);
  }
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
  else if (ackHandler.getId() == SAFETY)
    loopSafety();
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
  motorLeft.commandSpeed(propMsg.data[0]);
  motorRight.commandSpeed(propMsg.data[1]);
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
#ifdef HAS_ACTUATOR
  actuator.setDir(soufflanteCmdMsg.data);
#endif
}

void deadmanCallback()
{
  deadmanActive = deadmanMsg.data;

#ifdef HAS_MOTOR_PROP
  /*if (!deadmanActive && ackHandler.getId() == CONTROLLER)
  {
    // TODO : Test it
    motorLeft.disable();
    motorRight.disable();
  }*/
#endif
}

void estopCallback()
{
  digitalWrite(estopPin, estopMsg.data);
}

void lightCallback()
{
#ifdef HAS_LIGHTTOWER
  lightTower.toggle(lightMsg);
#endif
}

void pidCstCallback()
{
#ifdef HAS_MOTOR_PROP
  sendStatusWithMessage(INFO, MOTOR_PROP_DEVICE, "Setting PID for motor " + String(pidCstMsg.data[0]));

  if (pidCstMsg.data[0] = 0)
    motorLeft.setPID(pidCstMsg.data[1], pidCstMsg.data[2], pidCstMsg.data[3]);
    
  if (pidCstMsg.data[0] = 1)
    motorRight.setPID(pidCstMsg.data[1], pidCstMsg.data[2], pidCstMsg.data[3]);
#endif
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
    if (period > 75)
    {
      String msg = "Frequency was not met";
      msg += period;
      sendStatusWithMessage(FATAL, IMU_DEVICE, msg);
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

#ifdef HAS_GPS
    gps.getCoordinates(&gpsMsg);
    pbUtils.pbSend(1, GPS);
#endif

#ifdef HAS_ACTUATOR
    if (actuator.getCurrentPos() != soufflanteHeightMsg.data)
    {
      soufflanteHeightMsg.data = actuator.getCurrentPos();
      pbUtils.pbSend(1, SOUFFLANTE_HEIGHT);
    }
#endif

#ifdef HAS_MOTOR_PROP
    motorLeft.computePID();
    motorRight.computePID();

    encMsg.data_count = 2;
    encMsg.data[0] = motorLeft.getSpeed();
    encMsg.data[1] = motorRight.getSpeed();
    pbUtils.pbSend(1, ENC);

    debugMotMsg.data_count = 8;
    debugMotMsg.data[0] = encMsg.data[0];   // Vitesse
    debugMotMsg.data[1] = motorLeft.getCurrentCmd();
    debugMotMsg.data[2] = motorLeft.getCurrentOutput();
    debugMotMsg.data[3] = motorLeft.getDir();
    debugMotMsg.data[4] = encMsg.data[1];
    debugMotMsg.data[5] = motorRight.getCurrentCmd();
    debugMotMsg.data[6] = motorRight.getCurrentOutput();
    debugMotMsg.data[7] = motorRight.getDir();
    pbUtils.pbSend(1, DEBUG_MOT);
#endif
  }

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

          case PID_CST:
            pidCstCallback();
            break;
            
          case ESTOP:
          case LIGHT:
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
  }

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
          case ESTOP:
          case PID_CST:
            break;

          case DEADMAN:
            deadmanCallback();
            break;

          case LIGHT:
            lightCallback();
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

void loopSafety()
{
  bool state = digitalRead(estopStatePin);
  if (lastEstopState != state)
  {
    lastDebounceTime = millis();
  }

  if(state != estopState && (millis() - lastDebounceTime) > delayDebounceInterval)
  {
    estopState = state;
    estopStateMsg.data = state;
    pbUtils.pbSend(1, ESTOP_STATE);
  }

  lastEstopState = state;
  
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
          case SOUFFLANTE_CMD:
          case DEADMAN:
          case LIGHT:
          case PID_CST:
            break;
            
          case ESTOP:
            estopCallback();
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
