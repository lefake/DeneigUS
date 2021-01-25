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

// GPS
static double lat = 0;
static double lon = 0;
static double elev = 0;

// IMU
static double ddx = 0;
static double ddy = 0;
static double ddz = 0;
static double dx = 0;
static double dy = 0;
static double dz = 0;
static double mx = 0;
static double my = 0;
static double mz = 0;



/*
 * Fake posittion data
 */
void pos_msg_fake_data( std_msgs::Float32MultiArray* msg )
{
  // Set data
  msg->data[0] = x;
  msg->data[1] = y;
  msg->data[2] = z;

  // Update data
  x = (x > 50) ? -50 : x + 2;
  y = (y > 50) ? -50 : y + 1;
  z = (z > 50) ? -50 : z + 3;
}

/*
 * Fake obstacle position data
 */
void obs_pos_msg_fake_data( std_msgs::Float32MultiArray* msg )
{
  for (int i = 0; i < msg->data_length; i += NBS_DATA_SONRARS)
  {
    // Set data
    msg->data[i] = (obs == i) ? 0 : 1;
    msg->data[i+1] = (msg->data[i] > 0) ? dist : -1;

    // Update data
    dist = (dist + 1 > 100) ? 0 : dist + 1;
  }
  
  // Update data
  obs = (obs + NBS_DATA_SONRARS >= msg->data_length) ? 0 : obs + NBS_DATA_SONRARS;
}

/*
 * Fake EStop State data
 */
void estop_state_msg_fake_data( std_msgs::Float32MultiArray* msg )
{
  // Set data
  msg->data[0] = state;

  // Update data
  state = (state > 3) ? 0 : state + 1;
}

/*
 * Fake telemetrie battery data
 */
void tele_batt_msg_fake_data( std_msgs::Float32MultiArray* msg )
{ 
  for (int i = 0; i < msg->data_length; i += NBS_DATA_BATTERIES)
  {
    // Set data
    msg->data[i] = voltage;
    msg->data[i+1] = currant;
    msg->data[i+2] = temp;
    msg->data[i+3] = (pad == i) ? 0 : 1;

    // Update data
    voltage = (voltage > 15) ? 0 : voltage + 0.1;
    currant = (currant > 100) ? 0 : voltage + 1;
    temp = (temp > 30) ? -10 : temp + 0.1;
  }
  
  // Update data
  pad = (pad + NBS_DATA_BATTERIES >= msg->data_length) ? 0 : pad + NBS_DATA_BATTERIES;
}

/*
 * Fake Tourelle position data
 */
void pos_tourelle_msg_fake_data( std_msgs::Float32MultiArray* msg )
{
  // Set data
  yaw = (yaw > 180) ? 0 : yaw + 1;

  // Update data
  pitch = (pitch > 90) ? 0 : pitch + 1;
}

/*
 * Fake motor debug data
 */
void debug_mot_msg_fake_data( std_msgs::Float32MultiArray* msg )
{
  for (int i = 0; i < msg->data_length; i += NBS_DATA_MOTORS)
  {
    // Set data
    msg->data[i] = status;
    msg->data[i+1] = speed;
    msg->data[i+2] = torque;
    msg->data[i+3] = direction;
    msg->data[i+4] = position;

    // Update data
    status = (status > 0) ? 0 : 1;
    speed = (speed > 1) ? 0 : speed + 0.01;
    torque = (torque > 30) ? 0 : torque + 0.1;
    direction = (direction + 1 > 1) ? -1 : direction + 1;
    position = (position > 100) ? 0 : position + 1;
  }
}

/*
 * Fake gps data
 */
void gps_data_msg_fake_data( std_msgs::Float32MultiArray* msg )
{
  for (int i = 0; i < msg->data_length; i += NBS_DATA_GPS)
  {
    // Set data
    msg->data[i] = lat;
    msg->data[i+1] = lon;
    msg->data[i+2] = elev;

    // Update data
    lat = (lat > 20) ? -20 : lat + 1;
    lon = (lon > 25) ? -25 : lon + 2;
    elev = (elev > 30) ? -30 : elev + 5;
  }
}

/*
 * Fake imu data
 */
void imu_data_msg_fake_data( std_msgs::Float32MultiArray* msg )
{
  for (int i = 0; i < msg->data_length; i += NBS_DATA_IMU)
  {
    // Set data
    msg->data[i] = ddx;
    msg->data[i+1] = ddy;
    msg->data[i+2] = ddz;
    msg->data[i+3] = dx;
    msg->data[i+4] = dy;
    msg->data[i+5] = dz;
    msg->data[i+6] = mx;
    msg->data[i+7] = my;
    msg->data[i+8] = mz;

    // Update data
    ddx = (ddx > 10) ? -10 : ddx + 0.5;
    ddy = (ddy > 10) ? -10 : ddy + 0.4;
    ddz = (ddz > 10) ? -10 : ddz + 0.3;
    dx = (dx > 10) ? -10 : dx + 0.2;
    dy = (dy > 10) ? -10 : dy + 0.1;
    dz = (dz > 10) ? -10 : dz + 0.6;
    mx = (mx > 10) ? -10 : mx + 0.7;
    my = (my > 10) ? -10 : my + 0.8;
    mz = (mz > 10) ? -10 : mz + 0.9;
  }
}
