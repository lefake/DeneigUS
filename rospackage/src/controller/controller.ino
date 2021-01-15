/*
 * rosserial Publisher Example
 * Prints "hello world!"
 */

#include <ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>


// Function declaration
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


void cmd_vel_callback ( const geometry_msgs::Twist&  twistMsg )
{
  
}

void cmd_tourelle_callback ( const geometry_msgs::Twist&  twistMsg )
{
  
}

// GLOBALS

int val = 0;


void setup()
{
  nh.initNode();

  nh.subscribe(cmd_vel_sub);
  nh.subscribe(cmd_tourelle_sub);
  
  nh.advertise(pos_pub);
  nh.advertise(obs_pos_pub);
  nh.advertise(estop_state_pub);
  nh.advertise(tele_batt_pub);
  nh.advertise(pos_tourelle_pub);
  nh.advertise(debug_mot_pub);  


  pos_msg.data_length = 5;
  
  for (int i = 0; i < 5; ++i)
  {
    pos_msg.data[i] = 0;
  }
}

void loop()
{
  val = val + 1 > 255 ? 0 : val+1;
  
  pos_msg.data[0] = val;
  
  pos_pub.publish( &pos_msg );
  
  nh.spinOnce();
  delay(1000);
}
