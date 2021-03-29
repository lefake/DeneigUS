#ifndef TESTUTILS_H
#define TESTUTILS_H

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>
#include "constants.h"

class TestUtils
{
  public:
    TestUtils();
    void pos_msg_fake_data( geometry_msgs::Twist* msg );
    void obs_pos_msg_fake_data( std_msgs::Float32MultiArray* msg );
    void estop_state_msg_fake_data( std_msgs::Float32MultiArray* msg );
    void tele_batt_msg_fake_data( std_msgs::Float32MultiArray* msg );
    void pos_tourelle_msg_fake_data( std_msgs::Float32MultiArray* msg );
    void debug_mot_msg_fake_data( std_msgs::Float32MultiArray* msg );
    void gps_data_msg_fake_data( std_msgs::Float32MultiArray* msg );
    void imu_data_msg_fake_data( std_msgs::Float32MultiArray* msg );
    

  private:
    // Position
    double x = 0;
    double y = 5;
    double z = 10;
    
    // Sonar
    int obs = 0;
    int dist = 0;
    
    // EStop 
    int state = 0;
    
    // Battery
    double voltage = 0;
    double currant = 0;
    double temp = 0;
    int pad = 0;
    
    // Tourelle
    double yaw = 0;
    double pitch = 45;
    
    // Motors
    int status = 0;
    double speed = 0;
    double torque = 0;
    int direction = 0;
    double position = 0;
    
    // GPS
    double lat = 0;
    double lon = 0;
    double elev = 0;
    
    // IMU
    double ddx = 0;
    double ddy = 0;
    double ddz = 0;
    double dx = 0;
    double dy = 0;
    double dz = 0;
    double mx = 0;
    double my = 0;
    double mz = 0;
};

#endif //TESTUTILS_H
