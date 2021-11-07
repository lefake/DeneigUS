#!/usr/bin/env python3
import logging
from logging_utils import setup_logger, get_logger
from enum import Enum

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Int8, Bool
from sensor_msgs.msg import Joy

# Control mode values
class control_modes(Enum):
    stop = 0
    manual = 1
    auto = 2

supported_type = ["ps3", "ps4", "logi"]
def joy_button_mapper(joy_type):
    joy_indexes = {}
        
    if joy_type == "ps3":
        # Axes
        joy_indexes["prop_lin"] = 4
        joy_indexes["prop_ang"] = 3
        joy_indexes["chute_rot"] = 0
        joy_indexes["chute_elev"] = 1
        joy_indexes["soufl_speed"] = 5

        # Buttons
        joy_indexes["soufl_up"] = 2
        joy_indexes["soufl_down"] = 0
        joy_indexes["deadman"] = 4
        joy_indexes["switch_mode"] = 3

    elif joy_type == "ps4":
        # Axes
        joy_indexes["prop_lin"] = 4
        joy_indexes["prop_ang"] = 3
        joy_indexes["chute_rot"] = 0
        joy_indexes["chute_elev"] = 1
        joy_indexes["soufl_speed"] = 5

        # Buttons
        joy_indexes["soufl_up"] = 2
        joy_indexes["soufl_down"] = 0
        joy_indexes["deadman"] = 4
        joy_indexes["switch_mode"] = 3
    
    elif joy_type == "logi":
        # Axes
        joy_indexes["prop_lin"] = 4
        joy_indexes["prop_ang"] = 3
        joy_indexes["chute_rot"] = 0
        joy_indexes["chute_elev"] = 1
        joy_indexes["soufl_speed"] = 5

        # Buttons
        joy_indexes["soufl_up"] = 3
        joy_indexes["soufl_down"] = 0
        joy_indexes["deadman"] = 4
        joy_indexes["switch_mode"] = 2

    return joy_indexes


class Executif:
    def __init__(self, joy_indexes):
        self.logger = get_logger("executif.main")
        self.logger.debug("Started executif init")

        self.joy_indexes = joy_indexes
        self.stop_first_time_send = False
        self.last_control_mode = None
        self.control_mode = control_modes.stop

        # Publisher for robot's control
        self.prop_pub = rospy.Publisher('/prop', Float32MultiArray, queue_size=10)
        self.chute_pub = rospy.Publisher('/chute', Float32MultiArray, queue_size=10)
        self.soufflante_height_pub = rospy.Publisher('/soufflante_height', Int8, queue_size=10)
        self.control_mode_pub = rospy.Publisher('control_mode', Int8, queue_size=10)
        self.deadman_pub = rospy.Publisher('/deadman', Bool, queue_size=10)

        # Subscriber from nodes or robot
        rospy.Subscriber("/joy", Joy, self.joy_callback)
        rospy.Subscriber('/debug_arduino_data', Float32MultiArray, self.debug_arduino_data_callback)

        # Topics to be converted to the Arduinos
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

        self.logger.debug("Finished executif init")

    def cmd_vel_callback(self, msg):
        if self.control_mode == control_modes.auto:
            prop = Float32MultiArray()
            prop.data = self.lin_ang_2_tank(msg.linear.x, msg.angular.z)
            self.prop_pub.publish(prop)
        else:
            self.logger.debug(f"Cmd_vel recieved but ignored. Robot's mode is {self.control_mode}")

    def debug_arduino_data_callback(self, msg):
        self.logger.info(f"Debug arduino callback: {msg}")

    def joy_callback(self, msg):
        prop = Float32MultiArray()
        prop.data = [0, 0]
        chute = Float32MultiArray()
        chute.data = [0, 0, 0]
        soufflante_height = Int8()
        deadman = Bool()

        deadman.data = msg.buttons[self.joy_indexes["deadman"]]

        if not deadman.data:
            self.control_mode = control_modes.stop
        
        if self.control_mode == control_modes.stop and deadman.data:
            if self.last_control_mode is None:
                self.control_mode = control_modes.manual
                self.last_control_mode = self.control_mode
            else:
                self.control_mode = self.last_control_mode
            self.control_mode_pub.publish(self.control_mode.value)

        if not self.control_mode == control_modes.stop and msg.buttons[self.joy_indexes["switch_mode"]]:
            self.control_mode = control_modes.auto if self.control_mode == control_modes.manual else control_modes.manual
            self.last_control_mode = self.control_mode
            self.control_mode_pub.publish(self.control_mode.value)


        if self.control_mode == control_modes.manual:
            self.stop_first_time_send = True
            prop.data = self.lin_ang_2_tank(msg.axes[self.joy_indexes["prop_lin"]], msg.axes[self.joy_indexes["prop_ang"]])

            speed = abs(msg.axes[self.joy_indexes["soufl_speed"]] - 1) * 3 # TODO : Change the range
            chute.data = [msg.axes[self.joy_indexes["chute_rot"]], (msg.axes[self.joy_indexes["chute_elev"]]), speed]

            if msg.buttons[self.joy_indexes["soufl_up"]]:
                soufflante_height.data = 1
            elif msg.buttons[self.joy_indexes["soufl_down"]]:
                soufflante_height.data = -1
            else:
                soufflante_height.data = 0

            self.prop_pub.publish(prop)
            self.chute_pub.publish(chute)
            self.soufflante_height_pub.publish(soufflante_height)
            self.deadman_pub.publish(deadman)

        if self.control_mode == control_modes.auto:
            self.stop_first_time_send = True
            self.deadman_pub.publish(deadman)

        if self.control_mode == control_modes.stop and self.stop_first_time_send:
            prop = Float32MultiArray()
            prop.data = [0, 0]
            chute = Float32MultiArray()
            chute.data = [0, 0, 0]        # TODO : Fix angles shit 
            soufflante_height = Int8()

            self.prop_pub.publish(prop)
            self.chute_pub.publish(chute)
            self.soufflante_height_pub.publish(soufflante_height)
            self.control_mode_pub.publish(self.control_mode.value)
            self.deadman_pub.publish(deadman)

            self.stop_first_time_send = False

    def lin_ang_2_tank(self, lin, ang):
        # https://home.kendra.com/mauser/Joystick.html
        v = (1-abs(ang)) * (lin/1) + lin
        w = (1-abs(lin)) * (ang/1) + ang

        right_vel = (v+w) / 2
        left_vel = (v-w) / 2

        return [left_vel, right_vel]

if __name__ == "__main__":
    rospy.init_node('executif', anonymous=False)

    setup_logger(__file__, print_level=logging.INFO)
    logger = get_logger("executif")
    logger.info("Executif main Started")

    if not rospy.has_param("joy_type"):
        logger.fatal("Please set a param named joy_type")
    else:
        controller_type = rospy.get_param("joy_type")

        if controller_type in supported_type :
            Executif(joy_button_mapper(controller_type))
            rospy.spin()
        else:
            logger.fatal("Controller not supported")

    logger.info("Executif main Stopped")


