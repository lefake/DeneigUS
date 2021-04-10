#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/Range.h>

#include "TestUtils.h"
#include "RosUtils.h"
#include "IMU.h"
#include "Sonars.h"
#include "Gps.h"
#include "Motor.h"
#include "constants.h"
#include "pins.h"

// FUNCTION DECLARATION
void ros_init();
void ros_topic_init();
void ros_msg_init();


// ========== ROS ==========

ros::NodeHandle  nh;

// ROS IN
void cmd_vel_callback ( const geometry_msgs::Twist&  twistMsg );
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/cmd_vel", &cmd_vel_callback) ;
void cmd_tourelle_callback ( const geometry_msgs::Twist&  twistMsg );
ros::Subscriber<geometry_msgs::Twist> cmd_tourelle_sub("/cmd_tourelle", &cmd_tourelle_callback);

// ROS OUT
geometry_msgs::Twist pos_msg;
ros::Publisher pos_pub("/pos", &pos_msg);

sensor_msgs::Range obs_pos_msg;
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
float last_time = 0;
int val = 0;
int sonars_msg_seq = 0;

float vel_left = 0;
float vel_right = 0;

bool has_sonars = false;
bool has_imu = false;
bool has_gps = false;
bool has_motor = true;

TestUtils tests;
Sonars sonars;
IMU imu;
Gps gps;
Motor m_l, m_r;

void setup()
{
  // ROS
  ros_init();
  ros_msg_init();

  if(has_sonars)
  {
    sonars.init(sonars_trigger_pin, sonars_echo_pins);
    obs_pos_msg.radiation_type = sensor_msgs::Range::ULTRASOUND;
    obs_pos_msg.field_of_view = 0.26;
    obs_pos_msg.min_range = 0.0;
    obs_pos_msg.max_range = 400.0;
  }
  
  if (has_imu)
    imu.init();
  
  if (has_gps)
    gps.init();


  if (has_motor)
  {
    m_l.init(forw_left, back_left, pwm_left);
    m_r.init(forw_right, back_right, pwm_right);
  }
}

void loop()
{
  if (millis() - last_time > 1)
  {
    // ROS fake data
    tests.pos_msg_fake_data( &pos_msg );   
    tests.estop_state_msg_fake_data( & estop_state_msg);
    tests.tele_batt_msg_fake_data( &tele_batt_msg );
    tests.pos_tourelle_msg_fake_data( &pos_tourelle_msg );
    tests.debug_mot_msg_fake_data( &debug_mot_msg );



    if(has_sonars)
    {
      char frame_id[30] = "";
      for (int i = 0; i < NBS_SONARS; ++i)
      {
        (String("sonar_f_") + String(i)).toCharArray(frame_id, sizeof(frame_id));
        obs_pos_msg.range = sonars.dist(i);
        obs_pos_msg.header.seq = sonars_msg_seq++;
        obs_pos_msg.header.stamp = nh.now();
        obs_pos_msg.header.frame_id = frame_id;
        obs_pos_pub.publish( &obs_pos_msg );
      }
    }
//    else
//      tests.obs_pos_msg_fake_data( &obs_pos_msg );
  
    if(has_gps)
      gps.getCoordinates( &gps_data_msg );
    else
      tests.gps_data_msg_fake_data( &gps_data_msg );
    
    if(has_imu)
      imu.getValuesRos( &imu_data_msg );
    else
      tests.imu_data_msg_fake_data( &imu_data_msg );


    if (has_motor)
    {
      m_l.set_speed(vel_left);
      m_r.set_speed(vel_right);
    }

    // After 4 pubs and 2 subs it starts lagging
    // 

    // ROS pub
    pos_pub.publish( &pos_msg );  
    estop_state_pub.publish( &estop_state_msg );
    tele_batt_pub.publish( &tele_batt_msg );
    pos_tourelle_pub.publish( &pos_tourelle_msg );
    debug_mot_pub.publish( &debug_mot_msg );
    gps_data_pub.publish( &gps_data_msg );
    imu_data_pub.publish( &imu_data_msg );
    debug_arduino_data_pub.publish( &debug_arduino_data_msg );

    nh.spinOnce();

    last_time = millis();
  }
}

// CALLBACKS

void cmd_vel_callback ( const geometry_msgs::Twist&  twistMsg )
{
  float lin = twistMsg.linear.x;
  float rot = twistMsg.angular.z;
  
  vel_left = CALC(lin, -rot);
  vel_right = CALC(lin, rot);
}

void cmd_tourelle_callback ( const geometry_msgs::Twist&  twistMsg )
{

}

void ros_init()
{
  nh.getHardware()->setBaud(115200);
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
  RosUtils::init_twist( &pos_msg );
  RosUtils::init_range ( &obs_pos_msg );
  RosUtils::init_msg_array_values( &estop_state_msg, ESTOP_STATE_MSG_ARRAY_LEN );
  RosUtils::init_msg_array_values( &tele_batt_msg, TELE_BATT_MSG_ARRAY_LEN );
  RosUtils::init_msg_array_values( &pos_tourelle_msg, POS_TOURELLE_MSG_ARRAY_LEN );
  RosUtils::init_msg_array_values( &debug_mot_msg, DEBUG_MOT_MSG_ARRAY_LEN );
  RosUtils::init_msg_array_values( &gps_data_msg, GPS_DATA_MSG_ARRAY_LEN );
  RosUtils::init_msg_array_values( &imu_data_msg, IMU_DATA_MSG_ARRAY_LEN );
  RosUtils::init_msg_array_values( &debug_arduino_data_msg, DEBUG_ARDUINO_DATA_MSG_ARRAY_LEN );
}
