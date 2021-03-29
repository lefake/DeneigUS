#ifndef ROSUTILS_H
#define ROSUTILS_H

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <geometry_msgs/Twist.h>

class RosUtils
{
  public:
    static void init_msg_array_values ( std_msgs::Float32MultiArray* msg, int len );
    static void init_twist ( geometry_msgs::Twist* msg );
    
};

#endif //ROSUTILS_H
