#!/usr/bin/env python3
import logging
import os
from enum import Enum

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray
from deneigus.srv import trajgen
from sensor_msgs.msg import Joy
from logging_utils import setup_logger, get_logger

# Control mode values
class control_modes(Enum):
    stop = 0
    manual = 1
    auto = 2

class Executif:
    def __init__(self):
        self.logger = get_logger("executif.main")

        self.logger.debug("Started executif init")
        # Out
        self.pos_msg = Twist()
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pos_tourelle_msg = Twist()
        self.cmd_tourelle_pub = rospy.Publisher('/cmd_tourelle', Twist, queue_size=10)

        # In
        self.pos_sub = rospy.Subscriber('/pos', Twist, self.pos_callback)
        self.obs_pos_sub = rospy.Subscriber('/obs_pos', Float32MultiArray, self.obs_pos_callback)
        self.estop_state_sub = rospy.Subscriber('/estop_state', Float32MultiArray, self.estop_state_callback)
        self.tele_batt_sub = rospy.Subscriber('/tele_batt', Float32MultiArray, self.tele_batt_callback)
        self.pos_tourelle_sub = rospy.Subscriber('/pos_tourelle', Float32MultiArray, self.pos_tourelle_callback)
        self.debug_mot_sub = rospy.Subscriber('/debug_mot', Float32MultiArray, self.debug_mot_callback)
        self.gps_data_sub = rospy.Subscriber('/gps_data', Float32MultiArray, self.gps_data_callback)
        self.imu_data_sub = rospy.Subscriber('/imu_data', Float32MultiArray, self.imu_data_callback)
        self.joy_data_sub = rospy.Subscriber('/joy', Joy, self.joy_echo)
        self.imu_data_sub = rospy.Subscriber('/debug_arduino_data', Float32MultiArray, self.debug_arduino_data_callback)

        # Services
        self.traj_serv = rospy.ServiceProxy('/trajgen_srv', trajgen)

        # Variables
        self.ctl_mode = control_modes.stop

        self.logger.debug("Finished executif init")

    def joy_echo(self, msg):
        #self.logger.debug("Joy echo callback")
        # Random axies values
        # TODO : connect the actual joysticks values
        throttle = msg.axes(1)
        angle = msg.axes(0)

        if self.ctl_mode == control_modes.manuel:
            self.pos_msg.linear.x = throttle
            self.pos_msg.angular.z = angle

            self.cmd_vel_pub.publish(self.pos_msg)

    def pos_callback(self, msg):
        self.logger.debug("Pos callback")

    def obs_pos_callback(self, msg):
        self.logger.debug("Obs pas callback")

    def estop_state_callback(self, msg):
        self.logger.debug("EStop state callback")

    def tele_batt_callback(self, msg):
        self.logger.debug("Tele batt callback")

    def pos_tourelle_callback(self, msg):
        self.logger.debug("Pos tourelle callback")

    def debug_mot_callback(self, msg):
        self.logger.debug("Debug mot callback")

    def gps_data_callback(self, msg):
        self.logger.debug("GPS data callback")

    def imu_data_callback(self, msg):
        self.logger.debug("IMU data callback")

    def debug_arduino_data_callback(self, msg):
        self.logger.debug("Debug arduino callback")

if __name__ == "__main__":
    rospy.init_node('executif', anonymous=False)

    setup_logger(__file__, print_level=logging.INFO)
    logger = get_logger("executif")
    logger.info("Executif main Started")

    node = Executif()
    rospy.spin()

    logger.info("Executif main Stopped")


