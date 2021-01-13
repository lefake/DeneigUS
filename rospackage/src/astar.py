#!/usr/bin/env python3

import rospy
from deneigus.srv import trajgen

def server_answer (req):
    return req.input * 2

if __name__ == "__main__":
    rospy.init_node('astar', anonymous=False)
    s = rospy.Service('/trajgen_srv', trajgen, server_answer)
    rospy.spin()
