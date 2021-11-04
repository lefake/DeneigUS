#!/usr/bin/env python3

import logging
import rospy
from logging_utils import setup_logger, get_logger
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32MultiArray, UInt8, Int8

class JoyHandler:
    def __init__(self, joy_indexes):
        self.joy_indexes = joy_indexes
        self.deadman_off_latch = False

        rospy.Subscriber("/joy", Joy, self.joy_callback)

        self.prop_pub = rospy.Publisher('/prop', Float32MultiArray, queue_size=10)
        self.chute_pub = rospy.Publisher('/chute', Float32MultiArray, queue_size=10)
        self.soufl_pub = rospy.Publisher('/soufl', Int8, queue_size=10)
        self.deadman_pub = rospy.Publisher('/deadman', UInt8, queue_size=10)


    def joy_callback(self, msg):
        prop = Float32MultiArray()
        prop.data = [0, 0]
        chute = Float32MultiArray()
        chute.data = [0, 0, 0]
        soufl = Int8()
        deadman = UInt8()

        deadman.data = msg.buttons[self.joy_indexes["deadman"]]

        if deadman.data:
            self.deadman_off_latch = False
            prop.data = self.lin_ang_2_tank(msg.axes[self.joy_indexes["prop_lin"]], msg.axes[self.joy_indexes["prop_ang"]])

            speed = abs(msg.axes[self.joy_indexes["soufl_speed"]] - 1) * 3 # TODO : Change the range
            chute.data = [msg.axes[self.joy_indexes["chute_rot"]], (msg.axes[self.joy_indexes["chute_elev"]]), speed]

            if msg.buttons[self.joy_indexes["soufl_up"]]:
                soufl.data = 1
            elif msg.buttons[self.joy_indexes["soufl_down"]]:
                soufl.data = -1
            else:
                soufl.data = 0

        if self.deadman_off_latch:
            return

        if not deadman.data and not self.deadman_off_latch:
            self.deadman_off_latch = True

        
        self.prop_pub.publish(prop)
        self.chute_pub.publish(chute)
        self.soufl_pub.publish(soufl)
        self.deadman_pub.publish(deadman)


    def lin_ang_2_tank(self, lin, ang):
        # https://home.kendra.com/mauser/Joystick.html
        v = (1-abs(ang)) * (lin/1) + lin
        w = (1-abs(lin)) * (ang/1) + ang

        right_vel = (v+w) / 2
        left_vel = (v-w) / 2

        return [left_vel, right_vel]


def joy_button_mapper(joy_type):
    joy_indexes = {}
        
    if joy_type == "ps3":
        # Axes
        joy_indexes["prop_lin"] = 4
        joy_indexes["prop_ang"] = 3
        joy_indexes["chute_rot"] = 0
        joy_indexes["chute_elev"] = 1

        # Buttons
        joy_indexes["soufl_up"] = 13
        joy_indexes["soufl_down"] = 14
        joy_indexes["soufl_speed"] = 5
        joy_indexes["deadman"] = 4

    elif joy_type == "ps4":
        logger.error("Not yet supported")

    return joy_indexes


if __name__ == "__main__":
    rospy.init_node('joyHandler', anonymous=False)
    setup_logger(__file__, print_level=logging.INFO)
    logger = get_logger("joyHandler")

    supported_type = ["ps3", "ps4"]

    if not rospy.has_param("joy_type"):
        logger.fatal("Please set a param named joy_type")
    else:
        controller_type = rospy.get_param("joy_type")

        if controller_type in supported_type :
            JoyHandler(joy_button_mapper(controller_type))
            rospy.spin()
        else:
            logger.fatal("Controller not supported")
