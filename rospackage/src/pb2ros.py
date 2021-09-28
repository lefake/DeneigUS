#!/usr/bin/env python3

import serial
import logging

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy, Range

from logging_utils import setup_logger, get_logger
from PBUtils import PBSerialHandler, PBSerializationHandler, Topic

class PB2ROS:
    def __init__(self, serials):
        # TODO : Add Arduino ID acknowledge 
        self._logger = get_logger("pb2ros.main")
        self._logger.debug("Started pb2ros init")

        # Out Executif, In Arduino
        self._cmd_vel_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_vel_callback)
        self._cmd_tourelle_sub = rospy.Subscriber('/cmd_tourelle', Twist, self.cmd_tourelle_callback)

        # In Executif, Out Arduino
        self._debug_arduino_pub = rospy.Publisher('/debug_arduino_data', Float32MultiArray, queue_size=5)
        self._pos_pub = rospy.Publisher('/pos', Twist, queue_size=5)
        self._obs_pos_pub = rospy.Publisher('/obs_pos', Range, queue_size=5)

        self._estop_state_pub = rospy.Publisher('/estop_state', Float32MultiArray, queue_size=5)
        self._tele_batt_pub = rospy.Publisher('/tele_batt', Float32MultiArray, queue_size=5)
        self._pos_tourelle_pub = rospy.Publisher('/pos_tourelle', Float32MultiArray, queue_size=5)
        self._debug_mot_pub = rospy.Publisher('/debug_mot', Float32MultiArray, queue_size=5)
        self._gps_data_pub = rospy.Publisher('/gps_data', Float32MultiArray, queue_size=5)
        self._imu_data_pub = rospy.Publisher('/imu_data', Twist, queue_size=5)
        self._joy_data_pub = rospy.Publisher('/joy', Joy, queue_size=5)

        # Topic IDs much be the same in the Arduino enum (in constants.h)
        self._sub_topics = [
            Topic(1, self._cmd_vel_sub, dst="SENSORS"),
            Topic(2, self._cmd_tourelle_sub, dst="CONTROLLER"),
        ]
        self._pub_topics = [
            Topic(0, self._debug_arduino_pub),
            Topic(3, self._pos_pub),
            Topic(4, self._obs_pos_pub),
            Topic(5, self._estop_state_pub),
            Topic(6, self._tele_batt_pub),
            Topic(7, self._pos_tourelle_pub),
            Topic(8, self._debug_mot_pub),
            Topic(9, self._gps_data_pub),
            Topic(10, self._pos_pub), # Belle triche de dernière minute (self.pose_pub -> self.imu_data)
        ]
        self._topics = self._sub_topics + self._pub_topics
        self._serializer = PBSerializationHandler(self._topics)

        self._status_log_level_map = {
            1: self._logger.fatal,
            2: self._logger.error,
            3: self._logger.warning,
            4: self._logger.info,
            5: self._logger.debug
        }

        self._status_type_map = {
            0: "SERIAL_COMMUNICATION",
            1: "ENCODING_PB",
            2: "DECODING_PB",
            3: "TOPICS",
            4: "GPS_DEVICE",
            5: "IMU_DEVICE",
            6: "SONARS_DEVICE",
            7: "MOTOR_BLOW_DEVICE",
            8: "MOTOR_PROP_DEVICE",
            9: "SERVOS_DEVICE",
            10: "ENCODER_DEVICE",
            11: "ACKNOWLEDGE",
            12: "ID",
            13: "OTHER",
        }

        self._serials = []
        for i, s in enumerate(serials):
            self._serials.append(PBSerialHandler(s, i, self.new_msg_callback, self.new_status_callback, self._serializer))

        self._arduinos_acknowledged = False

    def new_msg_callback(self, response):
        self._logger.debug("Arduino msg in:" + str(response))
        msgs = self._serializer.deserialize(response)

        for msg in msgs:
            current_topic = next((topic for topic in self._topics if topic.id == msg[0]), None)
            if current_topic is not None:
                current_topic.pub.publish(current_topic.converter(msg[1]))
            else:
                self._logger.warn("new_msg_callback :  Couldn't find message ID")

    def new_status_callback(self, id, response):
        self._logger.debug("Arduino status in:" + str(response))
        status_values = response.decode()[1:-1].split(';')

        if len(status_values) == 2 and status_values[0] == '12':
            s = next((serial for serial in self._serials if serial.id == id), None)
            s.set_arduino_id(status_values[1])
            s.acknowledge_arduino("11")
            self._logger.info("Acknowledged : " + status_values[1])

        else:
            try:
                log_level = self._status_log_level_map[int(status_values[0])]
            except KeyError:
                self._logger.error("Log lvl not found for status msg")
                log_level = self._logger.error

            try:
                status_type = self._status_type_map[int(status_values[1])]
            except KeyError:
                self._logger.warning("Status type not found")
                status_type = "OTHER"

            log_level(id + " -> " + status_type + " : " + status_values[2])

    def cmd_vel_callback(self, msg):
        current_topic = next(topic for topic in self._topics if topic.name == "/cmd_vel")
        current_serial = next((serial for serial in self._serials if serial.id == current_topic.dst), None)

        if current_serial is None:
            self._logger.fatal("Arduino not acknowledged yet")
            return

        current_serial.write_pb_msg(current_topic.id, current_topic.converter(msg))

    def cmd_tourelle_callback(self, msg):
        current_topic = next(topic for topic in self._topics if topic.name == "/cmd_tourelle")
        current_serial = next((serial for serial in self._serials if serial.id == current_topic.dst), None)

        if current_serial is None:
            self._logger.fatal("Arduino not acknowledged yet")
            return

        current_serial.write_pb_msg(current_topic.id, current_topic.converter(msg))

    def kill(self):
        for s in self._serials:
            s.kill()

if __name__ == "__main__":
    # Add rospy.get_params() for the port and baudrate
    #serial.Serial('/dev/pts/4', 9600, timeout=0.05)
    arduinos = [
        serial.Serial('/dev/ttyUSB1', 115200, timeout=0.05),
        serial.Serial('/dev/ttyUSB2', 115200, timeout=0.05)
    ]
    rospy.init_node('pb2ros', anonymous=False)

    setup_logger(__file__, print_level=logging.INFO)
    logger = get_logger("pb2ros")
    logger.info("pb2ros main Started")

    pb2ros = PB2ROS(arduinos)
    rospy.spin()

    pb2ros.kill()
    logger.info("pb2ros main Stopped")
