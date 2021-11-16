#!/usr/bin/env python3

import rospy
import logging

from deneigus.srv import acknowledge,set_paths
from logging_utils import setup_logger, get_logger

import numpy as np


class StateManagement:
    def __init__(self):
        self.acknowledge_srv = rospy.Service('acknowledge', acknowledge, self.acknowledge_callback)
        self.set_paths_srv = rospy.Service('set_paths', set_paths, self.set_paths_callback)

        self.acknowledge_mbf = False
        self.acknowledge_soufflante = False

        self.path_mbf = []
        self.path_chute = []
        self.path_soufflante = []

    def acknowledge_callback(self, msg):
        # example: rosservice call /acknowledge "MBF" 1
        # example: rosservice call /acknowledge "Soufflante" 1
        if msg.finish==True:
            if msg.name=="Soufflante":
                self.acknowledge_soufflante = True
                success = True
            elif msg.name=="MBF":
                self.acknowledge_mbf = True
                success = True
            else:
                logger.info('Bad name')
                success = False

        else:
            success = False

        if self.acknowledge_soufflante and self.acknowledge_mbf:
            self.send_new_objectives()

        return success

    def send_new_objectives(self):
        logger.info('Send new objectives done')
        self.acknowledge_soufflante = False
        self.acknowledge_mbf = False
        return False

    def set_paths_callback(self, paths):
        # example: rosservice call /set_paths [0.0,1.0] [0.0,1.0] [0.0,0.0] [0]
        try:
            self.path_mbf = np.array([paths.mbf_x, paths.mbf_y])
            self.path_chute = paths.chute
            self.path_soufflante = paths.soufflante
            success = True
            logger.info('Set paths success')
        except:
            logger.info('Set path failed')
            success = False

        return success

if __name__ == '__main__':
    rospy.init_node('StateManagement', anonymous=False)

    setup_logger(__file__, print_level=logging.INFO)
    logger = get_logger("StateManagement")
    logger.info("StateManagement main Started")

    StateManagement()
    rospy.spin()

    logger.info("StateManagement main Stopped")