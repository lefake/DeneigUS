#!/usr/bin/env python3

import rospy
import logging

from deneigus.srv import acknowledge, set_paths
from deneigus.msg import chute_msg, mbf_msg
from logging_utils import setup_logger, get_logger
from geometry_msgs.msg import PoseStamped

def createMBF(x,y,z,xx,yy,zz,ww, v_max, v_min):
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

    msg = mbf_msg()
    msg.pose = p
    msg.v_max = v_max
    msg.v_min = v_min

    return msg

def createChuteMsg(x,y,force45):
    msg = chute_msg()
    msg.x = x
    msg.y = y
    msg.force45 = force45
    return msg

if __name__ == '__main__':
    rospy.init_node('simpleSetPath', anonymous=False)

    setup_logger(__file__, print_level=logging.INFO)
    logger = get_logger("simpleSetPath")
    logger.info("simpleSetPath main Started")

    path_mbf = []
    path_mbf.append(createMBF(11.25, 3, 0, 0, 0, 0, 1, 1.0, 0.1))
    path_mbf.append(createMBF(11.25, 3, 0, 0, 0, 0, 1, 1.0, 0.1))
    path_mbf.append(createMBF(11.25, 3, 0, 0, 0, 0.7071068, 0.7071068, 1.0, 0.1))
    path_mbf.append(createMBF(11.25, 3, 0, 0, 0, 0.7071068, 0.7071068, 1.0, 0.1))
    path_mbf.append(createMBF(11.25, 4, 0, 0, 0, 0.7071068, 0.7071068, 1.0, 0.1))
    path_mbf.append(createMBF(11.25, 4, 0, 0, 0, 0.7071068, 0.7071068, 1.0, 0.1))
    path_mbf.append(createMBF(11.25, 4, 0, 0, 0, 1, 0, 1.0, 0.1))
    path_mbf.append(createMBF(11.25, 4, 0, 0, 0, 1, 0, 1.0, 0.1))
    path_mbf.append(createMBF(3.5, 4, 0, 0, 0, 1, 0, 1.0, 0.1))
    path_mbf.append(createMBF(3.5, 4, 0, 0, 0, 1, 0, 1.0, 0.1))
    path_mbf.append(createMBF(3.5, 4, 0, 0, 0, 0.7071068, 0.7071068, 1.0, 0.1))
    path_mbf.append(createMBF(3.5, 4, 0, 0, 0, 0.7071068, 0.7071068, 1.0, 0.1))
    path_mbf.append(createMBF(3.5, 6.5, 0, 0, 0, 0.7071068, 0.7071068, 1.0, 0.1))
    path_mbf.append(createMBF(3.5, 6.5, 0, 0, 0, 0.7071068, 0.7071068, 1.0, 0.1))
    path_mbf.append(createMBF(3.5, 4, 0, 0, 0, 0.7071068, 0.7071068, 1.0, 0.1))
    path_mbf.append(createMBF(3.5, 4, 0, 0, 0, 0.7071068, 0.7071068, 1.0, 0.1))


    path_chute = []
    path_chute.append(createChuteMsg(0,0,False))
    path_chute.append(createChuteMsg(0,0,False))
    path_chute.append(createChuteMsg(0,0,False))
    path_chute.append(createChuteMsg(0,0,False))
    path_chute.append(createChuteMsg(0, 0, False))
    path_chute.append(createChuteMsg(0, 0, False))
    path_chute.append(createChuteMsg(0, 0, False))
    path_chute.append(createChuteMsg(0, 0, False))
    path_chute.append(createChuteMsg(0, 0, False))
    path_chute.append(createChuteMsg(0, 0, False))
    path_chute.append(createChuteMsg(0, 0, False))
    path_chute.append(createChuteMsg(0, 0, False))
    path_chute.append(createChuteMsg(0, 0, False))
    path_chute.append(createChuteMsg(0, 0, False))
    path_chute.append(createChuteMsg(0, 0, False))
    path_chute.append(createChuteMsg(0, 0, False))

    path_soufflante = []
    path_soufflante.append(-1)
    path_soufflante.append(1)
    path_soufflante.append(1)
    path_soufflante.append(-1)
    path_soufflante.append(-1)
    path_soufflante.append(1)
    path_soufflante.append(1)
    path_soufflante.append(-1)
    path_soufflante.append(-1)
    path_soufflante.append(1)
    path_soufflante.append(1)
    path_soufflante.append(-1)
    path_soufflante.append(-1)
    path_soufflante.append(1)
    path_soufflante.append(1)
    path_soufflante.append(-1)

    rospy.wait_for_service('set_paths')
    set_func = rospy.ServiceProxy('set_paths', set_paths)
    success = set_func(path_mbf, path_soufflante, path_chute)

    logger.info(success)

    logger.info("simpleSetPath main Stopped")