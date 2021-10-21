#!/usr/bin/env python3

import logging
import rospy
import time
import numpy as np

from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from logging_utils import setup_logger, get_logger

class TransformOdomMsg:
    def __init__(self):
        self.odom_sub = rospy.Subscriber('/odom_sim', Odometry, self.transform_callback)

        self.imu_enc_pub = rospy.Publisher('/odom_not_filtered', Odometry, queue_size=10)
        self.gps_pub = rospy.Publisher('/gps_data', NavSatFix, queue_size=10)

        self.lattitude_studio = 45.37767355737077 # rho
        self.longitude_studio = -71.92495310609306 # phi
        self.altitude_studio = 170

        self.x0, self.y0 = self.pol2cart(self.lattitude_studio , self.longitude_studio)
        self.z0 = self.altitude_studio

    def transform_callback(self, data):
        self.transform_gps(data)
        self.transform_imu_enc(data)


    def transform_gps(self, data):
        msg = NavSatFix()

        x = data.pose.pose.position.x + self.x0
        y = data.pose.pose.position.y + self.y0
        z = data.pose.pose.position.z + self.z0

        rho, phi = self.cart2pol(x,y)
        altitude = z

        msg.header = data.header
        #msg.status = -1
        msg.latitude = rho
        msg.longitude = phi
        msg.altitude = altitude

        self.gps_pub.publish(msg)

    def transform_imu_enc(self, data):
        msg = Odometry()

        msg.header = data.header
        msg.child_frame_id = data.child_frame_id
        msg.pose.pose.orientation = data.pose.pose.orientation
        msg.twist.twist.linear.x = data.twist.twist.linear.x
        msg.twist.twist.angular.z = data.twist.twist.angular.z

        self.imu_enc_pub.publish(msg)

    def cart2pol(self, x, y):
        rho = np.sqrt(x ** 2 + y ** 2)
        phi = np.arctan2(y, x)
        return (rho, phi)

    def pol2cart(self, rho, phi):
        x = rho * np.cos(phi)
        y = rho * np.sin(phi)
        return (x, y)


if __name__ == '__main__':
    rospy.init_node('transformOdomMsg', anonymous=False)

    setup_logger(__file__, print_level=logging.INFO)
    logger = get_logger("transformOdomMsg")
    logger.info("transformOdomMsg main Started")

    node = TransformOdomMsg()
    rospy.spin()

    logger.info("transformOdomMsg main Stopped")