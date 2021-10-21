import logging
import rospy
import time
import numpy as np

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from logging_utils import setup_logger, get_logger


class TransformOdomMsg:
    def __init__(self):
        self.pos_sub = rospy.Subscriber('/pos', Twist, self.transform_callback)

        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

    def transform_callback(self, msg):
        return


if __name__ == '__main__':
    rospy.init_node('transformOdomMsg', anonymous=False)

    setup_logger(__file__, print_level=logging.INFO)
    logger = get_logger("transformOdomMsg")
    logger.info("transformOdomMsg main Started")

    node = TransformOdomMsg()
    rospy.spin()

    logger.info("transformOdomMsg main Stopped")