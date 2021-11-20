#!/usr/bin/env python3

import logging
import rospy
import time
import os
import yaml

import numpy as np

from geographic_msgs.msg import GeoPose
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from logging_utils import setup_logger, get_logger
from pyproj import Proj
from robot_localization.srv import *

class TransformOdomMsg:
    def __init__(self):
        self.logger = get_logger("transformOdomMsg")
        self.logger.debug("Started transformOdomMsg init")

        self.enc_pub = rospy.Publisher('/wheel/odometry', Odometry, queue_size=10)
        self.imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
        self.gps_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)

        origin_path = os.getcwd() + '/../catkin_ws/src/deneigus/map/origin_sim.yaml'
        origin = self.load_yaml(origin_path)

        self.set_datum(origin)

        self.latitude_studio = origin['latitude']# rho
        self.longitude_studio = origin['longitude'] # phi
        self.altitude_studio = origin['altitude']

        self.p = Proj(init='epsg:3857', ellps='WGS84')

        self.scale = 1.42

        x0_no_offset, y0_no_offset = self.p(self.longitude_studio, self.latitude_studio) # longitude, lattitude
        self.x0 = x0_no_offset + origin['x_offset']*self.scale
        self.y0 = y0_no_offset + origin['y_offset']*self.scale
        self.z0 = self.altitude_studio + origin['z_offset']

        self.odom_sub = rospy.Subscriber('/odom_sim', Odometry, self.transform_callback)

    def load_yaml(self, path):
        with open(path, "r") as stream:
            try:
                data = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                logger.info(exc)

        return data

    def set_datum(self, origine):
        msg = GeoPose()

        msg.position.latitude = origine['latitude']
        msg.position.longitude = origine['longitude']
        msg.position.altitude = origine['altitude']

        msg.orientation.x = 0
        msg.orientation.y = 0
        msg.orientation.z = 0
        msg.orientation.w = 1

        rospy.wait_for_service('datum')
        try:
            geo_pose = rospy.ServiceProxy('datum', SetDatum)
            geo_pose(msg)
        except rospy.ServiceException as e:
            logger.info("Service call for datum failed: %s" % e)

    def transform_callback(self, data):
        self.transform_gps(data)
        self.transform_imu(data)
        self.transform_enc(data)


    def transform_gps(self, data):
        msg = NavSatFix()

        x = data.pose.pose.position.x*self.scale + self.x0
        y = data.pose.pose.position.y*self.scale + self.y0
        z = data.pose.pose.position.z + self.z0

        longitude, latitude = self.p(x,y, inverse=True)
        altitude = z

        msg.header.seq =  data.header.seq
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = 'base_link'
        msg.latitude = latitude
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