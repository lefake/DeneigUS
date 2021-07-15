#include <ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>

<<<<<<< Updated upstream
#include "TestUtils.h"
#include "RosUtils.h"
#include "IMU.h"
#include "Sonars.h"
#include "Gps.h"
=======

// -------------- TODO : USE #IFDEF TO REDUCE SKETCH SIZE -------------------- //


#include "PBUtils.h"
#include "twist.pb.h"
//#include "floatarray.pb.h"
#include "range.pb.h"

#include "Sonars.h"
#include "Motor.h"
//#include "IMU.h"
//#include "Gps.h"
>>>>>>> Stashed changes
#include "constants.h"
#include "pins.h"

// FUNCTION DECLARATION
void ros_init();
void ros_msg_init();

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
<<<<<<< Updated upstream
float last_time = 0;
int val = 0;

bool has_sonars = false;
bool has_imu = false;
bool has_gps = false;

TestUtils tests;
Sonars sonars;
IMU imu;
Gps gps;

void setup()
{
  // ROS
  ros_init();
  ros_msg_init();

=======
long last_time = 0;
long delay_interval = 10;

long last_time_sonar = 0;
long delay_interval_sonar = 100;

//FloatArray debug_arduino_msg = FloatArray_init_zero;
Twist cmd_vel_msg = Twist_init_zero;
Twist cmd_tourelle_msg = Twist_init_zero;
Twist pos_msg = Twist_init_zero;
Range obs_pos_msg = Range_init_zero;
//FloatArray imu_data_msg = FloatArray_init_zero;

Topic topics[] = {
      //{DEBUG_ARDUINO, FloatArray_fields, &debug_arduino_msg},
      {CMD_VEL, Twist_fields, &cmd_vel_msg},
      {CMD_TOURELLE, Twist_fields, &cmd_tourelle_msg},
      {POS, Twist_fields, &pos_msg},
      {OBS_POS, Range_fields, &obs_pos_msg},
      //{IMU_DATA, FloatArray_fields, &imu_data_msg},
      /*{ESTOP_STATE, FloatArray_fields, NULL},
      {TELE_BATT, FloatArray_fields, NULL},
      {POS_TOURELLE, FloatArray_fields, NULL},
      {DEBUG_MOT, FloatArray_fields, NULL},
      {GPS_DATA, FloatArray_fields, NULL},
      {IMU_DATA, FloatArray_fields, NULL},*/
    };

// PB Communication
int ind = 0;
char in_char;
boolean recv_in_progress = false;
bool in_cmd_complete = false;
int nbs_new_msgs = 0;
int new_msgs_ids[MAX_NBS_MSG];
char in_cmd[MAX_MSG_LEN];
PBUtils pbutils(topics, sizeof(topics) / sizeof(Topic));

// Motors
Motor m_l, m_r;
bool has_motor = true;
float vel_left = 0;
float vel_right = 0;

// Sonars
Sonars sonars;
bool has_sonars = true;
int sonars_msg_seq = 0;

// IMU
//IMU imu;
//bool has_imu = false;


// GPS
//Gps gps;
//bool has_gps = false;

void setup()
{
  //debug_arduino_msg.data_count = 2;
  
>>>>>>> Stashed changes
  if(has_sonars)
    sonars.init(sonars_trigger_pin, sonars_echo_pins);
  
<<<<<<< Updated upstream
  if (has_imu)
    imu.init();
  
  if (has_gps)
    gps.init();
=======
  if (has_motor)
  {
    m_l.init(forw_left, back_left, pwm_left);
    m_r.init(forw_right, back_right, pwm_right);
  }

  //if (has_imu)
  {
    //imu.init();
    //imu.calibrateGyroAcc();
    //imu_data_msg.data_count = IMU_DATA_MSG_ARRAY_LEN;
  }

  //if (has_gps)
  //  gps.init();
  
  Serial.begin(115200);

  // TODO : Add Arduino ID acknowledge 
>>>>>>> Stashed changes
}

void loop()
{
  if (millis() - last_time > 100)
  {
<<<<<<< Updated upstream
    // ROS fake data
    tests.pos_msg_fake_data( &pos_msg );   
    tests.estop_state_msg_fake_data( & estop_state_msg);
    tests.tele_batt_msg_fake_data( &tele_batt_msg );
    tests.pos_tourelle_msg_fake_data( &pos_tourelle_msg );
    tests.debug_mot_msg_fake_data( &debug_mot_msg );



    if(has_sonars)
      sonars.getDistancesRos( &obs_pos_msg );
    else
      tests.obs_pos_msg_fake_data( &obs_pos_msg );
  
    if(has_gps)
      gps.getCoordinates( &gps_data_msg );
    else
      tests.gps_data_msg_fake_data( &gps_data_msg );
=======
    // To create the Map TF in tf_broadcaster
    pbutils.pb_send(1, POS);
    
    // send imu
    //if (has_imu)
    {
      //imu.getValuesRos(&imu_data_msg, delay_interval);
      //pbutils.pb_send(1, IMU_DATA);
    }
>>>>>>> Stashed changes
    
    if(has_imu)
      imu.getValuesRos( &imu_data_msg );
    else
      tests.imu_data_msg_fake_data( &imu_data_msg );

      

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

    last_time = millis();
  }
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
