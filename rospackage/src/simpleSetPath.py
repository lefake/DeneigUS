#!/usr/bin/env python3

import rospy
import logging

from deneigus.srv import acknowledge,set_paths
from logging_utils import setup_logger, get_logger
from geometry_msgs.msg import PoseStamped

def createPose(x,y,z,xx,yy,zz,ww):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = rospy.get_rostime()
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.position.z = z
    p.pose.orientation.x = xx
    p.pose.orientation.y = yy
    p.pose.orientation.z = zz
    p.pose.orientation.w = ww
    return p

if __name__ == '__main__':
    rospy.init_node('simpleSetPath', anonymous=False)

    setup_logger(__file__, print_level=logging.INFO)
    logger = get_logger("simpleSetPath")
    logger.info("simpleSetPath main Started")

    path_mbf = []
    path_mbf.append(createPose(6,3,0, 0,0,0,1))
    path_mbf.append(createPose(10,3,0, 0,0,0,1))
    path_mbf.append(createPose(10,6,0, 0,0,1,0))

    path_chute = []
    path_chute.append(45)
    path_chute.append(45)
    path_chute.append(45)

    path_soufflante = []
    path_soufflante.append(-1)
    path_soufflante.append(-1)
    path_soufflante.append(-1)

    rospy.wait_for_service('set_paths')
    set_func = rospy.ServiceProxy('set_paths', set_paths)
    success = set_func(path_mbf, path_chute, path_soufflante)

    logger.info(success)

    logger.info("simpleSetPath main Stopped")