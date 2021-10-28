#!/usr/bin/env python3
import logging
from enum import Enum

import rospy
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import Range, NavSatFix
from nav_msgs.msg import Odometry
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
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cmd_vel_msg = Twist()
        self.cmd_tourelle_pub = rospy.Publisher('/cmd_tourelle', Twist, queue_size=10)
        self.cmd_tourelle_msg = Twist()
    
        # Used for Sim
        self.range_pub = rospy.Publisher('/range', Range, queue_size=10)
        self.coppelia_range_sub = rospy.Subscriber('/ranges_data', Float32MultiArray, self.coppelia_range_callback)

        # In
        self.pos_sub = rospy.Subscriber('/pos', Twist, self.pos_callback)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.estop_state_sub = rospy.Subscriber('/estop_state', Float32MultiArray, self.estop_state_callback)
        self.tele_batt_sub = rospy.Subscriber('/tele_batt', Float32MultiArray, self.tele_batt_callback)
        self.pos_tourelle_sub = rospy.Subscriber('/pos_tourelle', Float32MultiArray, self.pos_tourelle_callback)
        self.debug_mot_sub = rospy.Subscriber('/debug_mot', Float32MultiArray, self.debug_mot_callback)
        self.gps_data_sub = rospy.Subscriber('/gps/fix', NavSatFix, self.gps_data_callback)
        self.odom_not_filtered_sub = rospy.Subscriber('/odom/not_filtered', Odometry, self.odom_not_filtered_callback)
        self.joy_data_sub = rospy.Subscriber('/joy', Joy, self.joy_echo)
        self.debug_arduino_sub = rospy.Subscriber('/debug_arduino_data', Float32MultiArray, self.debug_arduino_data_callback)

        # Services
        self.traj_serv = rospy.ServiceProxy('/trajgen_srv', trajgen)

        # Variables
        self.ctl_mode = control_modes.manual

        self.logger.debug("Finished executif init")

    def odom_callback(self, msg):
        self.logger.debug("Odom callback")

    def joy_echo(self, msg):
        throttle_left = msg.axes[1]
        throttle_right = msg.axes[4]

        if self.ctl_mode == control_modes.manual:
            self.cmd_vel_msg.linear.x = throttle_left
            self.cmd_vel_msg.linear.y = throttle_right
            self.cmd_vel_pub.publish(self.cmd_vel_msg)


    def coppelia_range_callback(self, msg):
        #Building Range msg
        self.logger.debug("Range callback")
        self.r = Range()
        self.r.radiation_type = 0
        self.r.field_of_view = 0.3
        self.r.min_range = 0.02
        self.r.max_range = 2.0

        for x,_ in enumerate(msg.data):
            # Publishes Range from all sonars on same Topic
            self.r.header.stamp = rospy.Time.now()
            self.r.header.frame_id = "sonar_f_"+str(x)
            self.r.range = msg.data[x]
            self.range_pub.publish(self.r)

    def pos_callback(self, msg):
        self.logger.debug("Pos callback")

    def estop_state_callback(self, msg):
        self.logger.debug("EStop state callback")

    def tele_batt_callback(self, msg):
        self.logger.debug("Tele batt callback")

    def pos_tourelle_callback(self, msg):
        self.logger.debug("Pos tourelle callback")
        t = Float32MultiArray()
        t.data = msg.data
        self.cmd_vel_pub.publish(t)

    def debug_mot_callback(self, msg):
        self.logger.debug("Debug mot callback")

    def gps_data_callback(self, msg):
        self.logger.debug("GPS data callback")

    def odom_not_filtered_callback(self, msg):
        self.logger.debug("Odom_not_filtered_callback callback")

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


