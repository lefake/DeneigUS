from proto_gen_classes import floatarray_pb2, twist_pb2, int32_pb2

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Int32
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry


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
        pb = floatarray_pb2.FloatArray()
        pb.data.extend(ros.data)
        return pb

    @staticmethod
    def int32_pb2ros(pb):
        int32 = Int32()
        int32.data = pb.data
        return int32

    @staticmethod
    def int32_ros2pb(ros):
        int32 = int32_pb2.Int32()
        int32.data = ros.data
        return int32

    @staticmethod
    def enc_converter(pb):
        msg = Odometry()

        # Fuck

        msg.header.stamp = rospy.get_rostime()
        msg.child_frame_id = 'base_link'
        #msg.twist.twist.linear.x = pb.data[0].twist.twist.linear.x 
        #msg.twist.twist.angular.z = data.twist.twist.angular.z

        return msg

    @staticmethod
    def gps_converter(pb):
        msg = NavSatFix()

        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = 'base_link'
        msg.latitude = pb.data[0]
        msg.longitude = pb.data[1]
        msg.altitude = pb.data[2]

        return msg

    @staticmethod
    def imu_converter(pb):
        msg = Imu()

        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = 'base_link'
        msg.orientation.x = pb.data[0]
        msg.orientation.y = pb.data[1]
        msg.orientation.z = pb.data[2]
        msg.orientation.w = pb.data[3]

        return msg

default_converters = {"Float32MultiArray":(floatarray_pb2.FloatArray(), [MsgConverter.floatarray_pb2ros, MsgConverter.floatarray_ros2pb]),
                        "Twist":(twist_pb2.Twist(), [MsgConverter.twist_pb2ros, MsgConverter.twist_ros2pb]),
                        "Int32":(int32_pb2.Int32(), [MsgConverter.int32_pb2ros, MsgConverter.int32_ros2pb]),
                        }
