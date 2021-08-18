#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy, Range
import serial
import logging

from logging_utils import setup_logger, get_logger
from PBUtils import PBSerialHandler, PBSerializationHandler, Topic

class PB2ROS:
    def __init__(self, serials):
        # TODO : Add Arduino ID acknowledge 
        self.logger = get_logger("pb2ros.main")
        self.logger.debug("Started pb2ros init")

        # Out Executif, In Arduino
        self.cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self.cmd_tourelle_sub = rospy.Subscriber('/cmd_tourelle', Twist, self.cmd_tourelle_callback)

        # In Executif, Out Arduino
        self.debug_arduino_pub = rospy.Publisher('/debug_arduino_data', Float32MultiArray, queue_size=5)
        self.pos_pub = rospy.Publisher('/pos', Twist, queue_size=5)
        self.obs_pos_pub = rospy.Publisher('/obs_pos', Range, queue_size=5)

        self.estop_state_pub = rospy.Publisher('/estop_state', Float32MultiArray, queue_size=5)
        self.tele_batt_pub = rospy.Publisher('/tele_batt', Float32MultiArray, queue_size=5)
        self.pos_tourelle_pub = rospy.Publisher('/pos_tourelle', Float32MultiArray, queue_size=5)
        self.debug_mot_pub = rospy.Publisher('/debug_mot', Float32MultiArray, queue_size=5)
        self.gps_data_pub = rospy.Publisher('/gps_data', Float32MultiArray, queue_size=5)
        self.imu_data_pub = rospy.Publisher('/imu_data', Twist, queue_size=5)
        self.joy_data_pub = rospy.Publisher('/joy', Joy, queue_size=5)

        # Topic IDs much be the same in the Arduino enum (in constants.h)
        self._sub_topics = [
            Topic(1, self.cmd_vel_sub, dst=0),
            Topic(2, self.cmd_tourelle_sub, dst=0),
        ]
        self._pub_topics = [
            Topic(0, self.debug_arduino_pub),
            Topic(3, self.pos_pub),
            Topic(4, self.obs_pos_pub),
            Topic(5, self.estop_state_pub),
            Topic(6, self.tele_batt_pub),
            Topic(7, self.pos_tourelle_pub),
            Topic(8, self.debug_mot_pub),
            Topic(9, self.gps_data_pub),
            Topic(10, self.pos_pub), # Belle trice de dernière minute (self.pose_pub -> self.imu_data)
        ]
        self._topics = self._sub_topics + self._pub_topics
        self._msg_obj = [topic.obj for topic in self._topics]
        self._serializer = PBSerializationHandler(self._msg_obj)

        self._serials = []
        for s in serials:
            self._serials.append(PBSerialHandler(s, self.new_msg_callback, self._msg_obj))

    def new_msg_callback(self, response):
        logger.debug("Arduino in:" + str(response))
        msgs = self._serializer.deserialize(response)

        for msg in msgs:
            current_topic = next((topic for topic in self._topics if topic.id == msg[0]), None)
            if current_topic is not None:
                current_topic.pub.publish(current_topic.converter(msg[1]))
            else:
                self.logger.warn("new_msg_callback :  Couldn't find message ID")

    def new_id_callback(self, response):
        logger.debug("Arduino in:" + str(response))
        msgs = self._serializer.deserialize(response)

        #self._serials.add()

    def cmd_vel_callback(self, msg):
        obj = next(topic for topic in self._topics if topic.name == "/cmd_vel")
        self.logger.info("Welp")
        self._serials[obj.dst].write_pb_msg(obj.id, obj.converter(msg))

    def cmd_tourelle_callback(self, msg):
        obj = next(topic for topic in self._topics if topic.name == "/cmd_tourelle")
        self._serials[obj.dst].write_pb_msg(obj.id, obj.converter(msg))

    def kill(self):
        for s in self._serials:
            s.kill()

if __name__ == "__main__":
    # Add rospy.get_params() for the port and baudrate
    #arduino = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.05)
    arduinos = [
        serial.Serial('/dev/ttyUSB0', 115200, timeout=0.05)
    ]
    rospy.init_node('pb2ros', anonymous=False)

    setup_logger(__file__, print_level=logging.INFO)
    logger = get_logger("pb2ros")
    logger.info("pb2ros main Started")

    pb2ros = PB2ROS(arduinos)
    rospy.spin()

    pb2ros.kill()
    logger.info("pb2ros main Stopped")
