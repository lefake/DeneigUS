#!/usr/bin/env python3
import logging
from logging_utils import setup_logger, get_logger

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Int32, Bool
from sensor_msgs.msg import Joy

from common_utils import control_modes, behavior_choices, supported_type, joy_button_mapper, lin_ang_2_tank


class Ctrl_manuel:
    def __init__(self, joy_indexes):
        self.logger = get_logger("ctrl_manuel.main")
        self.logger.debug("Started ctrl_manuel init")

        self.joy_indexes = joy_indexes

        self.chute_pub = rospy.Publisher('/chute_manual', Float32MultiArray, queue_size=10)
        self.soufflante_cmd_pub = rospy.Publisher('/soufflante_cmd_manual', Int32, queue_size=10)
        self.prop_pub = rospy.Publisher('/prop_manual', Float32MultiArray, queue_size=10)

        rospy.Subscriber("/joy", Joy, self.joy_callback)

        self.logger.debug("Finished ctrl_manuel init")

    def joy_callback(self, msg):
        prop = Float32MultiArray()
        prop.data = [0, 0]
        chute = Float32MultiArray()
        chute.data = [0, 0, 0]
        soufflante_cmd = Int32()

        prop.data = lin_ang_2_tank(msg.axes[self.joy_indexes["prop_lin"]], msg.axes[self.joy_indexes["prop_ang"]])
        speed = abs(msg.axes[self.joy_indexes["soufl_speed"]] - 1) * 3  # TODO : Change the range
        chute.data = [(msg.axes[self.joy_indexes["chute_rot"]]) * 90,
                      (msg.axes[self.joy_indexes["chute_elev"]] + 1) * 45, speed]

        if msg.buttons[self.joy_indexes["soufl_up"]]:
            soufflante_cmd.data = 1
        elif msg.buttons[self.joy_indexes["soufl_down"]]:
            soufflante_cmd.data = -1
        else:
            soufflante_cmd.data = 0

        self.prop_pub.publish(prop)
        self.chute_pub.publish(chute)
        self.soufflante_cmd_pub.publish(soufflante_cmd)



class Executif:
    def __init__(self, joy_indexes):
        self.logger = get_logger("executif.main")
        self.logger.debug("Started executif init")

        self.joy_indexes = joy_indexes
        self.stop_first_time_send = False
        self.last_control_mode = None
        self.control_mode = control_modes.stop
        self.behavior_active = behavior_choices.get("BF")
        self.last_estop_state = 1
        self.light_value = 0
        self.light_color = 0
        self.last_servo_pos = Float32MultiArray()
        self.last_servo_pos.data = [0, 0, 0]

        Ctrl_manuel(joy_indexes)

        # Publisher for robot's control
        self.prop_auto_pub = rospy.Publisher('/prop_auto', Float32MultiArray, queue_size=10)
        self.prop_pub = rospy.Publisher('/prop', Float32MultiArray, queue_size=10)
        self.chute_pub = rospy.Publisher('/chute', Float32MultiArray, queue_size=10)
        self.soufflante_cmd_pub = rospy.Publisher('/soufflante_cmd', Int32, queue_size=10)
        self.control_mode_pub = rospy.Publisher('control_mode', Int32, queue_size=10)
        self.deadman_pub = rospy.Publisher('/deadman', Int32, queue_size=10)
        self.behavior_pub = rospy.Publisher('behavior', Int32, queue_size=10)
        self.estop_pub = rospy.Publisher('/estop', Int32, queue_size=10)
        self.light_pub = rospy.Publisher('/light', Float32MultiArray, queue_size=10)
        self.reset_pub = rospy.Publisher("/reset_plan", Bool, queue_size=10)

        # Subscriber from nodes or robot
        rospy.Subscriber("/joy", Joy, self.joy_callback)
        rospy.Subscriber("/chute_manual", Float32MultiArray, self.chute_callback, control_modes.manual)
        rospy.Subscriber("/chute_auto", Float32MultiArray, self.chute_callback, control_modes.auto)
        rospy.Subscriber("/soufflante_cmd_manual", Int32, self.soufflante_cmd_callback,control_modes.manual)
        rospy.Subscriber("/soufflante_cmd_auto", Int32, self.soufflante_cmd_callback, control_modes.auto)
        rospy.Subscriber("/prop_manual", Float32MultiArray, self.pose_callback, control_modes.manual)
        rospy.Subscriber("/prop_auto", Float32MultiArray, self.pose_callback, control_modes.auto)

        # Topics to be converted to the Arduinos
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)

        self.logger.debug("Finished executif init")

    # Convertion from MBF to tank drive
    def cmd_vel_callback(self, msg):
        prop = Float32MultiArray()
        prop.data = lin_ang_2_tank(msg.linear.x, msg.angular.z)
        self.prop_auto_pub.publish(prop)

    def debug_arduino_data_callback(self, msg):
        self.logger.info(f"Debug arduino callback: {msg}")

    def joy_callback(self, msg):
        deadman = Int32()
        estop = Int32()
        light = Float32MultiArray()

        estop.data = msg.buttons[self.joy_indexes["estop"]]
        if estop.data:
            self.last_estop_state = not self.last_estop_state
            estop.data = self.last_estop_state
            self.estop_pub.publish(estop)

        if msg.buttons[self.joy_indexes["light"]]:
            self.light_color = (self.light_color + 1) % 3
            self.light_value = (self.light_value + 1) % 2
            light.data = [self.light_color, self.light_value]
            self.light_pub.publish(light)

        # Cycle through all the behaviors -> TODO : make the choice dependant of the data from the interface
        if msg.buttons[self.joy_indexes["behavior_p"]]:
            self.behavior_active = (self.behavior_active + 1) % len(behavior_choices.keys())
        if msg.buttons[self.joy_indexes["behavior_m"]]:
            self.behavior_active = (self.behavior_active - 1) % len(behavior_choices.keys())

        if msg.buttons[self.joy_indexes["reset"]]:
            self.behavior_pub.publish(self.behavior_active)
            self.reset_pub.publish(True)

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

        if self.control_mode == control_modes.manual or self.control_mode == control_modes.auto:
            self.stop_first_time_send = True
            self.deadman_pub.publish(deadman)

        if self.control_mode == control_modes.stop and self.stop_first_time_send:
            prop = Float32MultiArray()
            prop.data = [0, 0]
            chute = Float32MultiArray()
            chute.data = [self.last_servo_pos.data[0], self.last_servo_pos.data[1], 0]  # Stop blowing but keep ori
            soufflante_cmd = Int32()

            self.prop_pub.publish(prop)
            self.chute_pub.publish(chute)
            self.soufflante_cmd_pub.publish(soufflante_cmd)
            self.control_mode_pub.publish(self.control_mode.value)
            self.deadman_pub.publish(deadman)

            self.stop_first_time_send = False

    def chute_callback(self, msg, control_mode):
        if self.control_mode == control_mode:
            self.chute_pub.publish(msg)
            self.last_servo_pos = msg

    def soufflante_cmd_callback(self, msg, control_mode):
        if self.control_mode == control_mode:
            self.soufflante_cmd_pub.publish(msg)

    def pose_callback(self, msg, control_mode):
        if self.control_mode == control_mode:
            self.prop_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node('executif', anonymous=False)

    setup_logger(__file__, print_level=logging.INFO)
    logger = get_logger("executif")
    logger.info("Executif main Started")

    if not rospy.has_param("joy_type"):
        logger.fatal("Please set a param named joy_type")
    else:
        controller_type = rospy.get_param("joy_type")

        if controller_type in supported_type:
            Executif(joy_button_mapper(controller_type))
            rospy.spin()
        else:
            logger.fatal("Controller not supported")

    logger.info("Executif main Stopped")
