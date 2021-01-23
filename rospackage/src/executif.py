#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from deneigus.srv import trajgen

logging = False

class Executif:
    def __init__(self):
        # Out
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.cmd_tourelle_pub = rospy.Publisher('/cmd_tourelle', Twist, queue_size=10)

        # In
        self.pos_sub = rospy.Subscriber('/pos', Float32MultiArray, self.pos_callback)
        self.obs_pos_sub = rospy.Subscriber('/obs_pos', Float32MultiArray, self.obs_pos_callback)
        self.estop_state_sub = rospy.Subscriber('/estop_state', Float32MultiArray, self.estop_state_callback)
        self.tele_batt_sub = rospy.Subscriber('/tele_batt', Float32MultiArray, self.telle_batt_callback)
        self.pos_tourelle_sub = rospy.Subscriber('/pos_tourelle', Float32MultiArray, self.pos_tourelle_callback)
        self.debug_mot_sub = rospy.Subscriber('/debug_mot', Float32MultiArray, self.debug_mot_callback)
        self.gps_data_sub = rospy.Subscriber('/gps_data', Float32MultiArray, self.gps_data_callback)
        self.imu_data_sub = rospy.Subscriber('/imu_data', Float32MultiArray, self.imu_data_callback)

        # Services
        self.traj_serv = rospy.ServiceProxy('/trajgen_srv', trajgen)

    def pos_callback(self, msg):
        if logging:
            rospy.loginfo('1 Hello from pos_callback with ' + str(msg.data[0]))

    def obs_pos_callback(self, msg):
        if logging:
            rospy.loginfo('2 obs_pos_callback with ' + str(msg.data[0]))

    def estop_state_callback(self, msg):
        if logging:
            rospy.loginfo('3 estop_state_callback with ' + str(msg.data[0]))

    def telle_batt_callback(self, msg):
        if logging:
            rospy.loginfo('4 telle_batt_callback with ' + str(msg.data[0]))

    def pos_tourelle_callback(self, msg):
        if logging:
            rospy.loginfo('5 pos_tourelle_callback with ' + str(msg.data[0]))

    def debug_mot_callback(self, msg):
        if logging:
            rospy.loginfo('6 pos_tourelle_callback with ' + str(msg.data[0]))

    def gps_data_callback(self, msg):
        if logging:
            rospy.loginfo('7 gps_data_callback with ' + str(msg.data[0]))

    def imu_data_callback(self, msg):
        if logging:
            rospy.loginfo('8 imu_data_callback with ' + str(msg.data[0]))

if __name__ == "__main__":
    rospy.init_node('executif', anonymous=False)
    node = Executif()
    rospy.spin()

