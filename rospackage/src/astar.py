#!/usr/bin/env python3

import logging
import rospy
from deneigus.srv import trajgen
from logging_utils import setup_logger, get_logger

def server_answer (req):
    return req.input * 2

if __name__ == "__main__":
    rospy.init_node('astar', anonymous=False)

    setup_logger(__file__)
    logger = get_logger("astar")
    logger.info("AStar main Started")

    s = rospy.Service('/trajgen_srv', trajgen, server_answer)
    rospy.spin()
    logger.debug("AStar main stopped")
