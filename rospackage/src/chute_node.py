#!/usr/bin/env python3

import rospy
import logging

from deneigus.msg import chute_msg
from logging_utils import setup_logger, get_logger
from std_msgs.msg import Float32MultiArray

from model_inverse import model_inverse


class ChuteNode:
    def __init__(self):
        self.logger = get_logger("ChuteNode")
        self.logger.debug("Started ChuteNode init")

        # TODO: load wind (vector) and snow density params
        self.v_wind = [0, 0]  # No wind
        self.ro_snow = 300  # Light snow

        self.chute_cmd_pub = rospy.Publisher('/chute', Float32MultiArray, queue_size=10)
        self.chute_new_goal_sub = rospy.Subscriber('/chute_new_goal', chute_msg, self.chute_goal_callback)

        # TODO: subscribe to odom and add rotation when diverging from path

    def chute_goal_callback(self, msg):
        x_target = msg.x
        y_target = msg.y
        force45 = msg.force45

        chute_cmd = Float32MultiArray()
        if x_target != 0.0 or y_target != 0.0:
            # chute_cmd.data = rotation_angle, elevation_angle, v_out
            chute_cmd.data = model_inverse(x_target, y_target, self.v_wind, self.ro_snow, force45)
            logger.info(chute_cmd.data)
            self.chute_cmd_pub.publish(chute_cmd)
        else:
            chute_cmd.data = [0,0,0]
            self.chute_cmd_pub.publish(chute_cmd)



if __name__ == '__main__':
    rospy.init_node('ChuteNode', anonymous=False)

    setup_logger(__file__, print_level=logging.INFO)
    logger = get_logger("ChuteNode")
    logger.info("ChuteNode main Started")

    ChuteNode()
    rospy.spin()

    logger.info("ChuteNode main Stopped")