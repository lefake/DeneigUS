#ifndef _CONFIGURATION_H
#define _CONFIGURATION_H

/*
 * This file is meant to have all the constants
 * concerning the physical configuration of the robot
 * e.i. : 
 *        Are any sonars connected. 
 *        Is the GPS connected. 
 *        etc...
 *        
 *  Uncomment the devices' names to enable them
 */

// ==================== GPS CONSTANTS =========================
 #define HAS_GPS

// ==================== IMU ===================================
#define HAS_IMU

// ==================== SONARS ================================
// #define HAS_SONARS

// ==================== MOTOR_BLOW ============================
 #define HAS_MOTOR_BLOW

// ==================== MOTOR_PROP ============================
 #define HAS_MOTOR_PROP

// ==================== SERVOS ================================
<<<<<<< HEAD
// #define HAS_SERVOS

// ==================== ENCODERS ==============================
// #define HAS_ENCODERS

=======
 #define HAS_SERVOS
>>>>>>> master


// ==================== STATE OF THE ROBOT ====================

// Uncomment to do a full calibration of the robot
// Will only calibrate sensors and write values to EEPROM
// #define CONFIGURATION_MODE

// #define DEBUGGING

#endif // _CONFIGURATION_H
