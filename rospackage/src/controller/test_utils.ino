#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

// Position
static double x = 0;
static double y = 5;
static double z = 10;

// Sonar
static int obs = 0;
static int dist = 0;

// EStop 
static int state = 0;

// Battery
static double voltage = 0;
static double currant = 0;
static double temp = 0;
static int pad = 0;

// Tourelle
static double yaw = 0;
static double pitch = 45;

// Motors
static int status = 0;
static double speed = 0;
static double torque = 0;
static int direction = 0;
static double position = 0;

void pos_msg_fake_data( std_msgs::Float32MultiArray* msg )
{
  msg->data[0] = x;
  msg->data[1] = y;
  msg->data[2] = z;
  
  x = (x > 50) ? -50 : x + 2;
  y = (y > 50) ? -50 : y + 1;
  z = (z > 50) ? -50 : z + 3;
}

void obs_pos_msg_fake_data( std_msgs::Float32MultiArray* msg )
{
  for (int i = 0; i < msg->data_length; i += 2)
  {
    msg->data[i] = (obs == i) ? 0 : 1;
    msg->data[i+1] = (obs > 0) ? dist : 0;
    
    dist = (dist + 1 > 100) ? 0 : dist + 1;
  }
  obs = (obs > OBS_POS_MSG_ARRAY_LEN) ? 0 : obs + 1;
}

void estop_state_msg_fake_data( std_msgs::Float32MultiArray* msg )
{
  msg->data[0] = state;
  state = (state > 3) ? 0 : state + 1;
}

void tele_batt_msg_fake_data( std_msgs::Float32MultiArray* msg )
{ 
  for (int i = 0; i < NBS_BATTERIES; ++i)
  {
    msg->data[i*NBS_DATA_BATTERIES] = voltage;
    msg->data[i*NBS_DATA_BATTERIES+1] = currant;
    msg->data[i*NBS_DATA_BATTERIES+2] = temp;
    msg->data[i*NBS_DATA_BATTERIES+3] = pad;

    voltage = (voltage > 15) ? 0 : voltage + 0.1;
    currant = (currant > 100) ? 0 : voltage + 1;
    temp = (temp > 30) ? -10 : temp + 0,1;
    pad = (pad > 0) ? 0 : 1;
  }
}

void pos_tourelle_msg_fake_data( std_msgs::Float32MultiArray* msg )
{
  yaw = (yaw > 180) ? 0 : yaw + 1;
  pitch = (pitch > 90) ? 0 : pitch + 1;
}

void debug_mot_msg_fake_data( std_msgs::Float32MultiArray* msg )
{
  for (int i = 0; i < NBS_MOTORS; ++i)
  {
    msg->data[i*NBS_DATA_MOTORS] = status;
    msg->data[i*NBS_DATA_MOTORS+1] = speed;
    msg->data[i*NBS_DATA_MOTORS+2] = torque;
    msg->data[i*NBS_DATA_MOTORS+3] = direction;
    msg->data[i*NBS_DATA_MOTORS+4] = position;

    status = (status > 0) ? 0 : 1;
    speed = (speed > 1) ? 0 : speed + 0.01;
    torque = (torque > 30) ? 0 : torque + 0,1;
    direction = (direction > 0) ? 0 : 1;
    position = (position > 100) ? 0 : position + 1;
  }
}
