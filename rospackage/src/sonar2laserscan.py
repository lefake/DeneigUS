#!/usr/bin/env python3

import logging
import rospy
import time
import numpy as np

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from logging_utils import setup_logger, get_logger
from tf import TransformListener

class Sonar2LaserScan:
    def __init__(self):
        self.pub = [
            rospy.Publisher('/laser_scan0', LaserScan, queue_size=10),
            rospy.Publisher('/laser_scan1', LaserScan, queue_size=10),
            rospy.Publisher('/laser_scan2', LaserScan, queue_size=10),
            rospy.Publisher('/laser_scan3', LaserScan, queue_size=10),
            rospy.Publisher('/laser_scan4', LaserScan, queue_size=10),
            rospy.Publisher('/laser_scan5', LaserScan, queue_size=10),
            rospy.Publisher('/laser_scan6', LaserScan, queue_size=10)
        ]

        self.range_sub = rospy.Subscriber('/sonar_pairs', Float32MultiArray, self.sonar_callback, queue_size=10)

        #self.listener = TransformListener()

        self.msg = LaserScan()
        self.msg.angle_min = -25*np.pi/180
        self.msg.angle_max = 25*np.pi/180
        self.msg.angle_increment = 1*np.pi/180
        self.msg.time_increment = 0
        self.msg.scan_time = 0
        self.msg.range_min = 0.02
        self.msg.range_max = 2.0

        self.nb_point_per_sonar = int(((self.msg.angle_max-self.msg.angle_min)/self.msg.angle_increment)/2)


    def sonar_callback(self, data):
        id = int(data.data[0])
        frame_id = 'sonar_f_' + str(id)

        range1 = data.data[1]
        range2 = data.data[2]

        if range1>=self.msg.range_max:
            range1 = float("inf")
        if range2>=self.msg.range_max:
            range2 = float("inf")

        #now = rospy.Time(0)
        #self.listener.waitForTransform("/base_link", "/map", now, rospy.Duration(0.1))

        self.msg.header.stamp = rospy.Time.now()#data.data[0]
        self.msg.header.frame_id =frame_id
        self.msg.ranges = [range1]*self.nb_point_per_sonar + [range2]*self.nb_point_per_sonar

        self.pub[id].publish(self.msg)


if __name__ == '__main__':
    rospy.init_node('Sonar2LaserScan', anonymous=False)

    setup_logger(__file__, print_level=logging.INFO)
    logger = get_logger("Sonar2LaserScan")
    logger.info("Sonar2LaserScan main Started")

    Sonar2LaserScan()
    rospy.spin()

    logger.info("Sonar2LaserScan main Stopped")