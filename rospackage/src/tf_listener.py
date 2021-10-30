#!/usr/bin/env python3

from math import pi
import rospy
from geometry_msgs.msg import Twist
import logging
from logging_utils import setup_logger, get_logger
import tf2_ros
import geometry_msgs.msg
import tf_conversions

def pos_callback(msg):
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    # TODO : Should be a Pose and not a Twist

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "map"
    t.child_frame_id = "base_link"
    t.transform.translation.x = msg.linear.x
    t.transform.translation.y = msg.linear.y
    t.transform.translation.z = 0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, msg.angular.z)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

if __name__ == "__main__":
    rospy.init_node('tf_handler')

    setup_logger(__file__, print_level=logging.INFO)
    logger = get_logger("tf_listener")
    logger.info("TFListener main Started")

    rospy.Subscriber('pos', Twist, pos_callback)
    rospy.spin()

    logger.info("TFListener main Stopped")
