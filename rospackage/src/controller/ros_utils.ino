#include <ros.h>
#include <std_msgs/Float32MultiArray.h>

void ros_init()
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
}

void init_msg_array_values ( std_msgs::Float32MultiArray* msg, int len )
{
  msg->data = (float *) malloc(sizeof(float)*len);
  msg->data_length = len;
  
  for (int i = 0; i < msg->data_length; ++i)
    msg->data[i] = 0.0;
}

void msg_init()
{
  init_msg_array_values( &pos_msg, POS_MSG_ARRAY_LEN);
  init_msg_array_values( &obs_pos_msg, OBS_POS_MSG_ARRAY_LEN );
  init_msg_array_values( &estop_state_msg, ESTOP_STATE_MSG_ARRAY_LEN );
  init_msg_array_values( &tele_batt_msg, TELE_BATT_MSG_ARRAY_LEN );
  init_msg_array_values( &pos_tourelle_msg, POS_TOURELLE_MSG_ARRAY_LEN );
  init_msg_array_values( &debug_mot_msg, DEBUG_MOT_MSG_ARRAY_LEN );
}
