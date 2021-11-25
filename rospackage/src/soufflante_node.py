#!/usr/bin/env python3

import rospy
import logging

from deneigus.srv import acknowledge,set_paths
from logging_utils import setup_logger, get_logger
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int32

import numpy as np

class SoufflanteNode:
    def __init__(self):
        self.logger = get_logger("SoufflanteNode")
        self.logger.debug("Started SoufflanteNode init")

        self.goal = 1
        self.goal_reach = True

        self.soufflante_cmd_pub = rospy.Publisher('/soufflante_cmd_auto', Int32, queue_size=10)
        self.soufflante_new_goal_sub = rospy.Subscriber('/soufflante_new_goal', Int32, self.soufflante_goal_callback)
        self.soufflante_height_sub = rospy.Subscriber('/soufflante_height', Int32, self.soufflante_height_callback)

        rospy.wait_for_service('acknowledge')
        path_func = rospy.ServiceProxy('acknowledge', acknowledge)
        path_func('Soufflante', 1)

    def soufflante_goal_callback(self, msg):
        if msg.data==0:
            rospy.wait_for_service('acknowledge')
            path_func = rospy.ServiceProxy('acknowledge', acknowledge)
            path_func('Soufflante', 1)
            self.goal_reach = True
        else:
            self.soufflante_cmd_pub.publish(msg.data)
            self.goal_reach = False
            self.goal = msg.data

    def soufflante_height_callback(self, msg):
        if self.goal_reach==False and msg.data==self.goal:
            rospy.wait_for_service('acknowledge')
            path_func = rospy.ServiceProxy('acknowledge', acknowledge)
            path_func('Soufflante', 1)
            self.goal_reach = True


if __name__ == '__main__':
    rospy.init_node('SoufflanteNode', anonymous=False)

    setup_logger(__file__, print_level=logging.INFO)
    logger = get_logger("SoufflanteNode")
    logger.info("SoufflanteNode main Started")

    SoufflanteNode()
    rospy.spin()

    logger.info("SoufflanteNode main Stopped")