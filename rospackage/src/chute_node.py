#!/usr/bin/env python3

import rospy
import logging
import tf
import numpy as np

from deneigus.msg import chute_msg
from deneigus.srv import acknowledge
from logging_utils import setup_logger, get_logger
from std_msgs.msg import Float32MultiArray, Float32
from nav_msgs.msg import Odometry

from model_inverse import model_inverse


class ChuteNode:
    def __init__(self):
        self.logger = get_logger("ChuteNode")
        self.logger.debug("Started ChuteNode init")

        # TODO: load wind (vector) and snow density params
        self.v_wind = [0, 0]  # No wind
        self.ro_snow = 300  # Light snow

        self.chute_cmd = Float32MultiArray()
        self.chute_cmd.data = [0,0,False]

        self.last_goal = chute_msg()

        self.rot_odom = 0
        self.rot_start = 0

        self.chute_cmd_pub = rospy.Publisher('/chute_auto', Float32MultiArray, queue_size=10)
        self.chute_new_goal_sub = rospy.Subscriber('/chute_new_goal', chute_msg, self.chute_goal_callback)
        self.odom_sub = rospy.Subscriber('/odometry/filtered/global', Odometry, self.odom_callback)
        self.soufflante_speed_sub = rospy.Subscriber('/soufflante_speed', Float32, self.soufflante_start_callback)

        rospy.wait_for_service('acknowledge')
        path_func = rospy.ServiceProxy('acknowledge', acknowledge)
        path_func('Chute', 1)

    def chute_goal_callback(self, msg):
        x_target = msg.x
        y_target = msg.y
        force45 = msg.force45

        # TODO: Rotation from cmd rather than odom
        # self.rot_start = self.rot_odom

        if msg == self.last_goal:
            rospy.wait_for_service('acknowledge')
            path_func = rospy.ServiceProxy('acknowledge', acknowledge)
            path_func('Chute', 1)

        elif x_target != 0.0 or y_target != 0.0:
            cmd_ele, cmd_rot, cmd_speed = model_inverse(x_target, y_target, self.v_wind, self.ro_snow, force45)
            self.chute_cmd.data = [np.rad2deg(cmd_ele), np.rad2deg(cmd_rot), cmd_speed]

        else:
            self.chute_cmd.data = [self.chute_cmd.data[0], self.chute_cmd.data[1], 0]

        self.last_goal = msg

    def odom_callback(self, msg):
        # TODO: add rotation when diverging from path
        # q = msg.pose.pose.orientation
        # self.rot_odom = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        # cmd = self.chute_cmd
        # new_cmd_rot = cmd.data[0] - (self.rot_start-self.rot_odom)
        # self.logger.info(new_cmd_rot)
        # if new_cmd_rot>np.deg2rad(80.0):
        #     new_cmd_rot = np.deg2rad(80.0)
        # elif new_cmd_rot<np.deg2rad(-80.0):
        #     new_cmd_rot = np.deg2rad(-80.0)

        #cmd.data = [new_cmd_rot, cmd.data[1], cmd.data[2]]
        self.chute_cmd_pub.publish(self.chute_cmd)

    def soufflante_start_callback(self, msg):
        # TODO: Acknowledge only if the goal is reach
        rospy.wait_for_service('acknowledge')
        path_func = rospy.ServiceProxy('acknowledge', acknowledge)
        path_func('Chute', 1)


if __name__ == '__main__':
    rospy.init_node('ChuteNode', anonymous=False)

    setup_logger(__file__, print_level=logging.INFO)
    logger = get_logger("ChuteNode")
    logger.info("ChuteNode main Started")

    ChuteNode()
    rospy.spin()

    logger.info("ChuteNode main Stopped")