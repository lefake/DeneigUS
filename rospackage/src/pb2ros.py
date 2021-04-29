#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy, Range
import serial
import logging

from logging_utils import setup_logger, get_logger
from PBUtils import PBSerialHandler, PBSerializationHandler, Topic
from proto_gen_classes import floatarray_pb2, twist_pb2, range_pb2


class PB2ROS:
    def __init__(self, serial):
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
        self.imu_data_pub = rospy.Publisher('/imu_data', Pose, queue_size=5)
        self.joy_data_pub = rospy.Publisher('/joy', Joy, queue_size=5)

        # Topic IDs much be the same in the Arduino enum (in constants.h)
        self._topics = [
            Topic(0, "/debug_arduino_data", floatarray_pb2.FloatArray(), self.floatarray_pb2ros, self.debug_arduino_pub),
            Topic(1, "/cmd_vel", twist_pb2.Twist(), self.twist_ros2pb),
            Topic(2, "/cmd_tourelle", twist_pb2.Twist(), self.twist_ros2pb),
            Topic(3, "/pos", twist_pb2.Twist(), self.twist_pb2ros, self.pos_pub),
            Topic(4, "/obs_pos", range_pb2.Range(), self.range_pb2ros, self.obs_pos_pub),
            Topic(5, "/estop_state", floatarray_pb2.FloatArray(), self.floatarray_pb2ros, self.estop_state_pub),
            Topic(6, "/tele_batt", floatarray_pb2.FloatArray(), self.floatarray_pb2ros, self.tele_batt_pub),
            Topic(7, "/pos_tourelle", floatarray_pb2.FloatArray(), self.floatarray_pb2ros, self.pos_tourelle_pub),
            Topic(8, "/debug_mot", floatarray_pb2.FloatArray(), self.floatarray_pb2ros, self.debug_mot_pub),
            Topic(9, "/gps_data", floatarray_pb2.FloatArray(), self.floatarray_pb2ros, self.gps_data_pub),
            Topic(10, "/imu_data", floatarray_pb2.FloatArray(), self.imu_pb2ros, self.imu_data_pub),
        ]
        self._msg_obj = [topic.obj for topic in self._topics]

        self._serial = PBSerialHandler(serial, self.new_msg_callback, self._msg_obj)
        self._serializer = PBSerializationHandler(self._msg_obj)

    def new_msg_callback(self, response):
        logger.debug("Arduino in:" + str(response))
        msgs = self._serializer.deserialize(response)

        for msg in msgs:
            current_topic = next((topic for topic in self._topics if topic.id == msg[0]), None)
            if current_topic is not None:
                current_topic.pub.publish(current_topic.converter(msg[1]))
            else:
                self.logger.warn("new_msg_callback :  Couldn't find message ID")

    def cmd_vel_callback(self, msg):
        obj = next(topic for topic in self._topics if topic.name == "/cmd_vel")
        self._serial.write_pb_msg(obj.id, obj.converter(msg))

    def cmd_tourelle_callback(self, msg):
        obj = next(topic for topic in self._topics if topic.name == "/cmd_tourelle")
        self._serial.write_pb_msg(obj.id, obj.converter(msg))

    def twist_pb2ros(self, pb):
        twist = Twist()

        twist.linear.x = pb.lx
        twist.linear.y = pb.ly
        twist.linear.z = pb.lz
        twist.angular.x = pb.ax
        twist.angular.y = pb.ay
        twist.angular.z = pb.az
        return twist

    def twist_ros2pb(self, ros):
        twist = twist_pb2.Twist()

        twist.lx = ros.linear.x
        twist.ly = ros.linear.y
        twist.lz = ros.linear.z
        twist.ax = ros.angular.x
        twist.ay = ros.angular.y
        twist.az = ros.angular.z
        return twist

    def floatarray_pb2ros(self, pb):
        fa = Float32MultiArray()
        # len
        fa.data = pb.data
        return fa

    def floatarray_ros2pb(self, ros):
        pb = floatarray_pb2.FloatArray
        pb.data = ros.data
        return pb

    def range_pb2ros(self, pb):
        ran = Range()
        ran.radiation_type = 0
        ran.header.seq = pb.seq
        ran.header.frame_id = pb.frame_id
        ran.header.stamp = rospy.Time.now()
        ran.field_of_view = 0.3
        ran.min_range = 0.1
        ran.max_range = 2
        ran.range = pb.range
        return ran

    def range_ros2pb(self, ros):
        pb = range_pb2.Range()
        #pb.header = ros.header
        pb.field_of_view = ros.field_of_view
        pb.min_range = ros.min_range
        pb.max_range = ros.max_range
        pb.range = ros.range
        return pb

    def imu_pb2ros(self, pb):
        pose = Pose()
        pose.position.x = pb.data[0]
        pose.position.y = pb.data[1]
        pose.orientation.x = pb.data[2]
        pose.orientation.y = pb.data[3]
        pose.orientation.z = pb.data[4]
        pose.orientation.w = pb.data[5]
        return pose

    def kill(self):
        self._serial.kill()


if __name__ == "__main__":
    # Add rospy.get_params() for the port and baudrate
    #arduino = serial.Serial('/dev/ttyUSB0', 115200, timeout=0.05)
    arduino = serial.Serial('/dev/pts/1', 9600, timeout=0.05)
    rospy.init_node('pb2ros', anonymous=False)

    setup_logger(__file__, print_level=logging.INFO)
    logger = get_logger("pb2ros")
    logger.info("pb2ros main Started")

    pb2ros = PB2ROS(arduino)
    rospy.spin()

    pb2ros.kill()
    logger.info("pb2ros main Stopped")