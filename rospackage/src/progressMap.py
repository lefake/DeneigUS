#!/usr/bin/env python3

import rospy
import logging
import imageio
import os
import numpy as np

from numpy import sin as s
from numpy import cos as c

from deneigus.srv import acknowledge,set_paths
from logging_utils import setup_logger, get_logger
from geometry_msgs.msg import PolygonStamped, PoseStamped
from std_msgs.msg import Float32, Int32, Bool, Float32MultiArray
from deneigus.msg import chute_msg, mbf_msg
from nav_msgs.msg import OccupancyGrid, Odometry
from nav_msgs.srv import GetMap

class ProgressMap:
    def __init__(self):
        self.logger = get_logger("ProgressMap")
        self.logger.debug("Started ChuteNode init")

        self.progress_map = OccupancyGrid()

        self.in_function = False

        self.souffl_offset_x = 0.4
        self.souffl_height = 0.3
        self.souffl_width = 0.69

        self.path = os.getcwd() + '/../catkin_ws/src/deneigus/map/progressMap.png'

        self.progress_map_pub = rospy.Publisher("/progress_map", OccupancyGrid, queue_size=5)

        rospy.wait_for_service('static_map')
        static_map_func = rospy.ServiceProxy('static_map', GetMap)
        self.static_map = static_map_func()

        self.progress_map = self.static_map.map

        self.map = np.reshape(self.progress_map.data, (self.progress_map.info.height, self.progress_map.info.width))

        rospy.Subscriber("/reset_plan", Bool, self.reset_callback)
        rospy.Subscriber("/chute", Float32MultiArray, self.chute_callback)
        rospy.Subscriber("/move_base_flex/global_costmap/footprint", PolygonStamped, self.footprint_callback)

    def reset_callback(self):
        self.logger.info("Reset")
        self.progress_map = self.static_map.map

    def chute_callback(self, msg):
        if msg.data[2]>0:
            self.in_function = True
        else:
            self.in_function = False

    def footprint_callback(self, msg):
        if self.in_function :
            self.progress_map.header.frame_id = msg.header.frame_id
            p = msg.polygon.points
            vertices = [(p[0].x, p[0].y), (p[1].x, p[1].y), (p[2].x, p[2].y), (p[3].x, p[3].y)]
            x_min = int(min([p[0].x, p[1].x, p[2].x, p[3].x])/self.progress_map.info.resolution)
            x_max = int(max([p[0].x, p[1].x, p[2].x, p[3].x])/self.progress_map.info.resolution)
            y_min = int(min([p[0].y, p[1].y, p[2].y, p[3].y])/self.progress_map.info.resolution)
            y_max = int(max([p[0].y, p[1].y, p[2].y, p[3].y])/self.progress_map.info.resolution)

            for x in range(x_min, x_max+1):
                for y in range(y_min, y_max+1):
                    in_footprint = self.test_point((x + 0.5) * self.progress_map.info.resolution,
                                                    (y + 0.5) * self.progress_map.info.resolution,
                                                    vertices)
                    if in_footprint:
                        self.map[y,x] = 50

        self.progress_map.data = tuple(self.map.flatten())
        self.progress_map_pub.publish(self.progress_map)

        np_data = np.array([self.color_converter(e) for e in self.progress_map.data])
        reshaped = np.flipud(np.reshape(np_data, (self.progress_map.info.height, self.progress_map.info.width))[25:75,25:125])
        imageio.imwrite(self.path, reshaped, format='png', optimize=False, quantize=4)

    def is_on_right_side(self, x, y, xy0, xy1):
        x0, y0 = xy0
        x1, y1 = xy1
        a = float(y1 - y0)
        b = float(x0 - x1)
        c = - a * x0 - b * y0
        return a * x + b * y + c >= 0

    def test_point(self, x, y, vertices):
        num_vert = len(vertices)
        is_right = [self.is_on_right_side(x, y, vertices[i], vertices[(i + 1) % num_vert]) for i in range(num_vert)]
        all_left = not any(is_right)
        all_right = all(is_right)
        return all_left or all_right

    def color_converter(self, value):
        if value == -1:
            return 0xcd / 0xff
        return (100 - value) / 100.0


if __name__ == '__main__':
    rospy.init_node('ProgressMap', anonymous=False)

    setup_logger(__file__, print_level=logging.INFO)
    logger = get_logger("ProgressMap")
    logger.info("ProgressMap main Started")

    ProgressMap()
    rospy.spin()

    logger.info("ProgressMap main Stopped")