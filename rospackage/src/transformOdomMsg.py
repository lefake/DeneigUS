#!/usr/bin/env python3

import logging
import rospy
import time
import numpy as np

from sensor_msgs.msg import NavSatFix, Imu#
from nav_msgs.msg import Odometry
from logging_utils import setup_logger, get_logger
from pyproj import Proj

class TransformOdomMsg:
    def __init__(self):
        self.odom_sub = rospy.Subscriber('/odom_sim', Odometry, self.transform_callback)

        self.enc_pub = rospy.Publisher('/wheel/odometry', Odometry, queue_size=10)
        self.imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
        self.gps_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)

        self.lattitude_studio = 45.37767355737077 # rho
        self.longitude_studio = -71.92495310609306 # phi
        self.altitude_studio = 170

        self.p = Proj(init='epsg:3857', ellps='WGS84')

        self.x0, self.y0 = self.p(self.longitude_studio, self.lattitude_studio) # longitude, lattitude
        self.z0 = self.altitude_studio

        self.scale = 1.42

    def transform_callback(self, data):
        self.transform_gps(data)
        self.transform_imu(data)
        self.transform_enc(data)


    def transform_gps(self, data):
        msg = NavSatFix()

        x = data.pose.pose.position.x*self.scale + self.x0
        y = data.pose.pose.position.y*self.scale + self.y0
        z = data.pose.pose.position.z + self.z0

        longitude, lattitude = self.p(x,y, inverse=True)
        altitude = z

        msg.header.seq =  data.header.seq
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = 'base_link'
        msg.latitude = lattitude
        msg.longitude = longitude
        msg.altitude = altitude

        self.gps_pub.publish(msg)

    def transform_enc(self, data):
        msg = Odometry()

        msg.header.seq =  data.header.seq
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = 'base_link'
        msg.twist.twist.linear.x = data.twist.twist.linear.x
        msg.twist.twist.linear.y = data.twist.twist.linear.y
        msg.twist.twist.linear.z = data.twist.twist.linear.z
        msg.twist.twist.angular.z = data.twist.twist.angular.z

        self.enc_pub.publish(msg)

    def transform_imu(self, data):
        msg = Imu()

        msg.header.seq =  data.header.seq
        msg.header.stamp = rospy.get_rostime()#data.header.stamp
        msg.header.frame_id = 'base_link'
        msg.orientation = data.pose.pose.orientation

        self.imu_pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('transformOdomMsg', anonymous=False)

    setup_logger(__file__, print_level=logging.INFO)
    logger = get_logger("transformOdomMsg")
    logger.info("transformOdomMsg main Started")

    node = TransformOdomMsg()
    rospy.spin()

    logger.info("transformOdomMsg main Stopped")