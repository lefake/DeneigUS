#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>

#include "TestUtils.h"
#include "RosUtils.h"
#include "Sonars.h"
#include "constants.h"
#include "pins.h"

// FUNCTION DECLARATION
void ros_init();
void ros_msg_init();

TestUtils tests;

// ========== ROS ==========

ros::NodeHandle  nh;

// ROS IN
void cmd_vel_callback ( const geometry_msgs::Twist&  twistMsg );
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", &cmd_vel_callback) ;
void cmd_tourelle_callback ( const geometry_msgs::Twist&  twistMsg );
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

std_msgs::Float32MultiArray gps_data_msg;
ros::Publisher gps_data_pub("/gps_data", &gps_data_msg);

std_msgs::Float32MultiArray imu_data_msg;
ros::Publisher imu_data_pub("/imu_data", &imu_data_msg);


// Debug
std_msgs::Float32MultiArray debug_arduino_data_msg;
ros::Publisher debug_arduino_data_pub("/debug_arduino_data", &debug_arduino_data_msg);


// GLOBALS

Sonars sonars;
int val = 0;

void setup()
{
  // ROS
  ros_init();
  ros_msg_init();

  sonars.init(sonars_trigger_pin, sonars_echo_pins);
}

void loop()
{
  // ROS fake data
  tests.pos_msg_fake_data( &pos_msg );
  // tests.obs_pos_msg_fake_data( &obs_pos_msg );
  tests.estop_state_msg_fake_data( & estop_state_msg);
  tests.tele_batt_msg_fake_data( &tele_batt_msg );
  tests.pos_tourelle_msg_fake_data( &pos_tourelle_msg );
  tests.debug_mot_msg_fake_data( &debug_mot_msg );
  tests.gps_data_msg_fake_data( &gps_data_msg );
  tests.imu_data_msg_fake_data( &imu_data_msg );
  
  sonars.getDistancesRos( &obs_pos_msg );

  // ROS pub
  pos_pub.publish( &pos_msg );
  obs_pos_pub.publish( &obs_pos_msg );
  estop_state_pub.publish( &estop_state_msg );
  tele_batt_pub.publish( &tele_batt_msg );
  pos_tourelle_pub.publish( &pos_tourelle_msg );
  debug_mot_pub.publish( &debug_mot_msg );
  gps_data_pub.publish( &gps_data_msg );
  imu_data_pub.publish( &imu_data_msg );
  debug_arduino_data_pub.publish( &debug_arduino_data_msg );

  
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

// 
void ros_init()
{
  nh.initNode();

  // Subs
  nh.subscribe(cmd_vel_sub);
  nh.subscribe(cmd_tourelle_sub);

  // Pubs
  nh.advertise(pos_pub);
  nh.advertise(obs_pos_pub);
  nh.advertise(estop_state_pub);
  nh.advertise(tele_batt_pub);
  nh.advertise(pos_tourelle_pub);
  nh.advertise(debug_mot_pub);
  nh.advertise(gps_data_pub);
  nh.advertise(imu_data_pub);
  nh.advertise(debug_arduino_data_pub);
}

void ros_msg_init()
{
  // Init all messages
  RosUtils::init_msg_array_values( &pos_msg, POS_MSG_ARRAY_LEN);
  RosUtils::init_msg_array_values( &obs_pos_msg, OBS_POS_MSG_ARRAY_LEN );
  RosUtils::init_msg_array_values( &estop_state_msg, ESTOP_STATE_MSG_ARRAY_LEN );
  RosUtils::init_msg_array_values( &tele_batt_msg, TELE_BATT_MSG_ARRAY_LEN );
  RosUtils::init_msg_array_values( &pos_tourelle_msg, POS_TOURELLE_MSG_ARRAY_LEN );
  RosUtils::init_msg_array_values( &debug_mot_msg, DEBUG_MOT_MSG_ARRAY_LEN );
  RosUtils::init_msg_array_values( &gps_data_msg, GPS_DATA_MSG_ARRAY_LEN );
  RosUtils::init_msg_array_values( &imu_data_msg, IMU_DATA_MSG_ARRAY_LEN );
  RosUtils::init_msg_array_values( &debug_arduino_data_msg, DEBUG_ARDUINO_DATA_MSG_ARRAY_LEN );
}
