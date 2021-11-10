#!/usr/bin/env python3

import serial
import logging
from msg_utils import *
from proto_gen_classes import floatarray_pb2

import rospy
from std_msgs.msg import Float32MultiArray, Int32
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry

from logging_utils import setup_logger, get_logger
from PBUtils import *

class PB2ROS:
    def __init__(self, serials):
        # TODO : Add Arduino ID acknowledge 
        self._logger = get_logger("pb2ros.main")
        self._logger.debug("Started pb2ros init")

        # Out Executif, In Arduino
        self._prop_sub = rospy.Subscriber('/prop', Float32MultiArray, self.sub_callback, ('/prop'))
        self._chute_sub = rospy.Subscriber('/chute', Float32MultiArray, self.sub_callback, ('/chute'))
        self._soufflante_height_sub = rospy.Subscriber('/soufflante_height', Int32, self.sub_callback, ('/soufflante_height'))
        self._deadman_sub = rospy.Subscriber('/deadman', Int32, self.sub_callback, ('/deadman'))

        # In Executif, Out Arduino
        self._debug_arduino_pub = rospy.Publisher('/debug_arduino_data', Float32MultiArray, queue_size=10)
        self._enc_pub = rospy.Publisher('/wheel/odometry', Odometry, queue_size=10)
        self._gps_pub = rospy.Publisher('/gps/fix', NavSatFix, queue_size=10)
        self._imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
        self._range_pairs_pub = rospy.Publisher('/range_pairs', Float32MultiArray, queue_size=10)
        self._estop_state_pub = rospy.Publisher('/estop_state', Int32, queue_size=10)
        
        # Topic IDs much be the same in the Arduino enum (in constants.h)
        self._pub_topics = [
            PubTopic(0, self._debug_arduino_pub),
            PubTopic(1, self._enc_pub, floatarray_pb2.FloatArray(), MsgConverter.enc_converter), # Might need to remove () on FloatArray
            PubTopic(2, self._imu_pub, floatarray_pb2.FloatArray(), MsgConverter.imu_converter),
            PubTopic(3, self._gps_pub, floatarray_pb2.FloatArray(), MsgConverter.gps_converter),
            PubTopic(4, self._range_pairs_pub),
            PubTopic(5, self._estop_state_pub),
        ]
        self._sub_topics = [
            SubTopic(6, self._prop_sub, ["CONTROLLER"]),
            SubTopic(7, self._chute_sub, ["CONTROLLER"]),
            SubTopic(8, self._soufflante_height_sub, ["CONTROLLER"]),
            SubTopic(9, self._deadman_sub, ["CONTROLLER"]),
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
            11: "OTHER",
        }

        self._serials = []
        for i, s in enumerate(serials):
            self._serials.append(PBSerialHandler(s, self.new_msg_callback, self.new_status_callback, self._serializer))

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

    def sub_callback(self, msg, curent_topic_name):
        current_topic = next((topic for topic in self._topics if topic.name == curent_topic_name), None)     # Catch if no topic
        current_serial = next((serial for serial in self._serials if serial.id in current_topic.dst), None)

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
        serial.Serial('/dev/pts/7', 9600, timeout=0.05)
        #serial.Serial('/dev/ttyUSB0', 115200, timeout=0.05),
        #serial.Serial('/dev/ttyUSB2', 115200, timeout=0.05)
    ]
    rospy.init_node('pb2ros', anonymous=False)

    setup_logger(__file__, print_level=logging.INFO)
    logger = get_logger("pb2ros")
    logger.info("pb2ros main Started")

    pb2ros = PB2ROS(arduinos)
    rospy.spin()

    pb2ros.kill()
    logger.info("pb2ros main Stopped")
