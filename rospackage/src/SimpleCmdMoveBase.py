#!/usr/bin/env python3

import logging
from logging_utils import setup_logger, get_logger

import rospy

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose
from tf.transformations import quaternion_from_euler


class SimpleCmdMoveBase:
    def __init__(self):
        client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        client.wait_for_server()

        q = quaternion_from_euler(0, 0, 0)
        p = Pose()
        p.position.x = 1.0
        p.position.y = 0.0
        p.position.z = 0.0
        p.orientation.x = q[0]
        p.orientation.y = q[1]
        p.orientation.z = q[2]
        p.orientation.w = q[3]

        logger.info(p)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = p

        client.send_goal(goal)
        wait = client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            logger.info("Allo")
            return client.get_result()


if __name__ == "__main__":
    rospy.init_node('SimpleCmdMoveBase', anonymous=False)

    setup_logger(__file__, print_level=logging.INFO)
    logger = get_logger("SimpleCmdMoveBase")
    logger.info("SimpleCmdMoveBase main Started")

    SimpleCmdMoveBase()
    rospy.spin()

    logger.info("SimpleCmdMoveBase main Stopped")