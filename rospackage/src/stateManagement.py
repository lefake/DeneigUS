#!/usr/bin/env python3

import rospy
import logging

from deneigus.srv import acknowledge,set_paths
from logging_utils import setup_logger, get_logger
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, Int32
from deneigus.msg import chute_msg, mbf_msg

import numpy as np


class StateManagement:
    def __init__(self):
        self.logger = get_logger("ChuteNode")
        self.logger.debug("Started StateManagement init")

        self.acknowledge_srv = rospy.Service('acknowledge', acknowledge, self.acknowledge_callback)
        self.set_paths_srv = rospy.Service('set_paths', set_paths, self.set_paths_callback)

        self.acknowledge_mbf = False
        self.acknowledge_soufflante = False
        self.acknowledge_chute = False

        self.path_mbf = []
        self.path_chute = []
        self.path_soufflante = []

        self.len_path = 0

        self.mbf_pub = rospy.Publisher('/mbf_new_goal', mbf_msg, queue_size=10)
        self.chute_pub = rospy.Publisher('/chute_new_goal', chute_msg, queue_size=10)
        self.soufflante_pub = rospy.Publisher('/soufflante_new_goal', Int32, queue_size=10)
        self.progress_pub = rospy.Publisher('/progress_bar', Float32, queue_size=10)

    def acknowledge_callback(self, msg):
        # example: rosservice call /acknowledge "MBF" 1
        # example: rosservice call /acknowledge "Soufflante" 1
        if msg.finish==True:
            logger.info(f'Got acknowledge from {msg.name}')
            if msg.name=='Soufflante':
                self.acknowledge_soufflante = True
                success = True
            elif msg.name=='MBF':
                self.acknowledge_mbf = True
                success = True
            elif msg.name=='Chute':
                self.acknowledge_chute = True
                success = True
            else:
                logger.info('Bad name')
                success = False

        else:
            success = False

        if len(self.path_mbf) != 0:
            self.send_new_objectives()

        return success

    def send_new_objectives(self):
        if self.acknowledge_soufflante and self.acknowledge_mbf and self.acknowledge_chute:
            self.mbf_pub.publish(self.path_mbf.pop(0))
            self.chute_pub.publish(self.path_chute.pop(0))
            self.soufflante_pub.publish(self.path_soufflante.pop(0))

            progress = len(self.path_mbf)/self.len_path*100
            self.progress_pub.publish(progress)

            self.acknowledge_soufflante = False
            self.acknowledge_mbf = False
            self.acknowledge_chute = False

            logger.info('Send new objectives done')

    def set_paths_callback(self, paths):
        try:
            self.path_mbf = paths.mbf
            self.path_chute = list(paths.chute)
            self.path_soufflante = list(paths.soufflante)

            self.len_path = len(self.path_mbf)

            success = True
            self.logger.info('Set paths success')
        except:
            self.logger.info('Set path failed')
            success = False

        self.send_new_objectives()

        return success

if __name__ == '__main__':
    rospy.init_node('StateManagement', anonymous=False)

    setup_logger(__file__, print_level=logging.INFO)
    logger = get_logger("StateManagement")
    logger.info("StateManagement main Started")

    StateManagement()
    rospy.spin()

    logger.info("StateManagement main Stopped")