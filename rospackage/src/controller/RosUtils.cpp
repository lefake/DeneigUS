#include "RosUtils.h"

void RosUtils::init_msg_array_values ( std_msgs::Float32MultiArray* msg, int len )
{
  //Allocate the array data
  msg->data = (float *) malloc(sizeof(float)*len);
  msg->data_length = len;

  //Initialize every value to 0
  for (int i = 0; i < msg->data_length; ++i)
    msg->data[i] = 0.0;
}

void RosUtils::init_twist ( geometry_msgs::Twist* msg )
{
  msg = (geometry_msgs::Twist *) malloc(sizeof(geometry_msgs::Twist));
}

void RosUtils::init_range ( sensor_msgs::Range* msg )
{
  msg = (sensor_msgs::Range *) malloc(sizeof(sensor_msgs::Range));
}
