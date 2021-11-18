#!/usr/bin/env python3

import rospy
import logging

from deneigus.srv import acknowledge,set_paths
from logging_utils import setup_logger, get_logger
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Int8, Int32

import numpy as np

class SoufflanteNode:
    def __init__(self):

        # TODO: cmd=0 don't move
        self.last_height = 0
        self.new_height = 0

        self.goal_reach = False

        self.soufflante_cmd_pub = rospy.Publisher('/soufflante_cmd', Int32, queue_size=10)
        self.soufflante_new_goal_sub = rospy.Subscriber('/soufflante_new_goal', Int8, self.soufflante_goal_callback)
        self.soufflante_height_sub = rospy.Subscriber('/soufflante_height', Int8, self.soufflante_height_callback)

        rospy.wait_for_service('acknowledge')
        path_func = rospy.ServiceProxy('acknowledge', acknowledge)
        path_func('Soufflante', 1)

    def soufflante_goal_callback(self, msg):
        self.last_height = self.new_height
        self.new_height = msg.data

        if self.new_height==self.last_height:
            rospy.wait_for_service('acknowledge')
            path_func = rospy.ServiceProxy('acknowledge', acknowledge)
            path_func('Soufflante', 1)
            self.goal_reach = True
        else:
            self.soufflante_cmd_pub.publish(self.new_height)
            self.goal_reach = False

    def soufflante_height_callback(self, msg):
        if self.goal_reach==False and msg.data==self.new_height:
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