#!/usr/bin/env python3

import logging
import rospy
import time
import numpy as np

from sensor_msgs.msg import PointCloud2, Range, PointField
from logging_utils import setup_logger, get_logger

class Sonar2PC():
    def __init__(self):
        self.point_cloud_pub = rospy.Publisher('/point_cloud', PointCloud2, queue_size=10)
        self.range_sub = rospy.Subscriber('/range', Range, self.sonar_callback, queue_size=10)

        self.point_density = 1 * np.pi/180 # points/d

        self.id = 0


    def sonar_callback(self, data):
        if  data.min_range < data.range < data.max_range:
            # Point cloud msg
            msg = PointCloud2()

            # Call the transform function to convert data range from sonar to points in 3D
            points = self.transform_data(data)

            # header
            msg.header.frame_id = data.header.frame_id
            msg.header.seq = self.id
            self.id += 1
            msg.header.stamp = data.header.stamp

            # Number of points
            N = len(points)
            msg.height = 1
            msg.width = N

            # Message field [x, y, z] in float (4 bytes)
            msg.fields = [
                PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1)
            ]
            msg.is_bigendian = False

            # Number of byte for each point [x, y, z] * 4bytes = 12 bytes
            msg.point_step = 12

            # Number of byte in total
            msg.row_step = msg.point_step * N
            msg.is_dense = True

            # Convert points in string for pointcloud msg
            msg.data = points.tostring()

            self.point_cloud_pub.publish(msg)

    def transform_data(self, data):
        theta_list = np.arange(-data.field_of_view/2, data.field_of_view/2, self.point_density)
        list_points = []
        x = data.range
        for theta in theta_list:
            y = data.range * np.tan(theta)
            list_points.append([x, y, 0])

        # Converting list to array
        array_points = np.array(list_points, dtype=np.float32)

        return array_points


if __name__ == '__main__':
    rospy.init_node('sonar2pc', anonymous=False)

    setup_logger(__file__, print_level=logging.INFO)
    logger = get_logger("sonar2pc")
    logger.info("sonar2pc main Started")

    node = Sonar2PC()
    rospy.spin()

    logger.info("Sonar2PC main Stopped")