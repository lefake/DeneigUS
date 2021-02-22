#ifndef ROSUTILS_H
#define ROSUTILS_H

#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

class RosUtils
{
  public:
    static void init_msg_array_values ( std_msgs::Float32MultiArray* msg, int len );
    
};

#endif //ROSUTILS_H
