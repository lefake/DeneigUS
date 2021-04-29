#!/usr/bin/env python3

import rospy
import tf
from geometry_msgs.msg import Twist, Pose
import math
import logging
from logging_utils import setup_logger, get_logger
import tf2_ros
import geometry_msgs.msg
import tf_conversions
from nav_msgs.msg import MapMetaData


class TfListener:
    def __init__(self):
        self._res = 1       # Default resolution, updated by map_metadata_callback

        rospy.Subscriber('/imu_data', Pose, self.pos_callback)
        rospy.Subscriber('/map_metadata', MapMetaData, self.map_metadata_callback)

        # Sur un ou deux node ?
        # Twist ou Pose
        # Encodeur vs GPS vs IMU
        # Fusion de capteur

        self.listen()

    def listen(self):
        listener = tf.TransformListener()
        while not rospy.is_shutdown():
            try:
                listener.waitForTransform('/map', '/robot', rospy.Time(), rospy.Duration(4.0))
                (trans, rot) = listener.lookupTransform('/map', '/robot', rospy.Time(0))

                vel_pub = rospy.Publisher('/vel_robot', Twist, queue_size=1)
                msg = Twist()
                msg.linear.x = math.floor(trans[0] / self._res)
                msg.linear.y = math.floor(trans[1] / self._res)
                vel_pub.publish(msg)

                logger.debug("Pos in map : x(" + str(msg.linear.x) + "), y(" + str(msg.linear.y) + ")")
            except Exception as e:
                logger.error(e)


    def pos_callback(self, msg):
        br = tf2_ros.TransformBroadcaster()
        t = geometry_msgs.msg.TransformStamped()

        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "robot"
        t.transform.translation.x = msg.position.x
        t.transform.translation.y = msg.position.y
        t.transform.translation.z = msg.position.z
        t.transform.rotation.x = msg.orientation.x
        t.transform.rotation.y = msg.orientation.y
        t.transform.rotation.z = msg.orientation.z
        t.transform.rotation.w = msg.orientation.w

        br.sendTransform(t)

    def map_metadata_callback(self, msg):
        self._res = round(msg.resolution, 5)

if __name__ == "__main__":
    rospy.init_node('tf_handler')

    setup_logger(__file__, print_level=logging.INFO)
    logger = get_logger("tf_listener")
    logger.info("TFListener main Started")

    try:
        TfListener()
    except rospy.ROSInterruptException:
        pass

    logger.info("TFListener main Stopped")
