from proto_gen_classes import floatarray_pb2, twist_pb2, range_pb2

import rospy
from geometry_msgs.msg import Twist, Pose
from rospy.topics import Message
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Joy, Range

class MsgFactory:
    def __init__(self):
        self.msg_map = {"Float32MultiArray":(floatarray_pb2.FloatArray(), [MsgConverter.floatarray_pb2ros, MsgConverter.floatarray_ros2pb]),
                        "Twist":(twist_pb2.Twist(), [MsgConverter.twist_pb2ros, MsgConverter.twist_ros2pb]),
                        "Range":(range_pb2.Range(), [MsgConverter.range_pb2ros, MsgConverter.range_ros2pb]),
                        }

    def getMsg(self, type):
        return self.msg_map[type]


class MsgConverter:
    @staticmethod
    def twist_pb2ros(pb):
        twist = Twist()

        twist.linear.x = pb.lx
        twist.linear.y = pb.ly
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = pb.az
        return twist

    @staticmethod
    def twist_ros2pb(ros):
        twist = twist_pb2.Twist()

        twist.lx = ros.linear.x
        twist.ly = ros.linear.y
        twist.az = ros.angular.z
        return twist

    @staticmethod
    def floatarray_pb2ros(pb):
        fa = Float32MultiArray()
        fa.data = pb.data
        return fa

    @staticmethod
    def floatarray_ros2pb(ros):
        pb = floatarray_pb2.FloatArray
        pb.data_count = len(ros.data)
        pb.data = ros.data
        return pb

    @staticmethod
    def range_pb2ros(pb):
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

    @staticmethod
    def range_ros2pb(ros):
        pb = range_pb2.Range()
        #pb.header = ros.header
        pb.field_of_view = ros.field_of_view
        pb.min_range = ros.min_range
        pb.max_range = ros.max_range
        pb.range = ros.range
        return pb

    @staticmethod
    def imu_pb2ros(pb):
        pose = Pose()
        pose.position.x = pb.data[0]
        pose.position.y = pb.data[1]
        pose.orientation.x = pb.data[2]
        pose.orientation.y = pb.data[3]
        pose.orientation.z = pb.data[4]
        pose.orientation.w = pb.data[5]
        return pose