#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include "constants.h"

// FUNCTION DECLARATION ros_utils

void ros_init();
void msg_init();
void init_msg_array_values ( std_msgs::Float32MultiArray* msg, int len );

// FUNCTION DECLARATION test_utils

void pos_msg_fake_data( std_msgs::Float32MultiArray* );
void obs_pos_msg_fake_data( std_msgs::Float32MultiArray* );
void estop_state_msg_fake_data( std_msgs::Float32MultiArray* );
void tele_batt_msg_fake_data( std_msgs::Float32MultiArray* );
void pos_tourelle_msg_fake_data( std_msgs::Float32MultiArray* );
void debug_mot_msg_fake_data( std_msgs::Float32MultiArray* );

void cmd_vel_callback ( const geometry_msgs::Twist&  twistMsg );
void cmd_tourelle_callback ( const geometry_msgs::Twist&  twistMsg );

// ========== ROS ==========

ros::NodeHandle  nh;

// ROS IN
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", &cmd_vel_callback) ;
ros::Subscriber<geometry_msgs::Twist> cmd_tourelle_sub("/cmd_tourelle", &cmd_tourelle_callback);

// ROS OUT
std_msgs::Float32MultiArray pos_msg;
ros::Publisher pos_pub("/pos", &pos_msg);

std_msgs::Float32MultiArray obs_pos_msg;
ros::Publisher obs_pos_pub("/obs_pos", &obs_pos_msg);

std_msgs::Float32MultiArray estop_state_msg;
ros::Publisher estop_state_pub("/estop_state", &estop_state_msg);

std_msgs::Float32MultiArray tele_batt_msg;
ros::Publisher tele_batt_pub("/tele_batt", &tele_batt_msg);

std_msgs::Float32MultiArray pos_tourelle_msg;
ros::Publisher pos_tourelle_pub("/pos_tourelle", &pos_tourelle_msg);

std_msgs::Float32MultiArray debug_mot_msg;
ros::Publisher debug_mot_pub("/debug_mot", &debug_mot_msg);

// GLOBALS

int val = 0;


void setup()
{
  ros_init();
  msg_init();
}

void loop()
{
  pos_msg_fake_data( &pos_msg );
  obs_pos_msg_fake_data( &obs_pos_msg );
  estop_state_msg_fake_data( & estop_state_msg);
  tele_batt_msg_fake_data( &tele_batt_msg );
  pos_tourelle_msg_fake_data( &pos_tourelle_msg );
  debug_mot_msg_fake_data( &debug_mot_msg );
  
  pos_pub.publish( &pos_msg );
  obs_pos_pub.publish( &obs_pos_msg );
  estop_state_pub.publish( &estop_state_msg );
  tele_batt_pub.publish( &tele_batt_msg );
  pos_tourelle_pub.publish( &pos_tourelle_msg );
  debug_mot_pub.publish( &debug_mot_msg );
  
  nh.spinOnce();
  delay(1000);
}

// CALLBACKS

void cmd_vel_callback ( const geometry_msgs::Twist&  twistMsg )
{
  
}

void cmd_tourelle_callback ( const geometry_msgs::Twist&  twistMsg )
{
  
}
