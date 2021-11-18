
import logging
import math

from logging_utils import setup_logger, get_logger
import rospy
import yaml
import os
import tf

from std_msgs.msg import Float32MultiArray, Int32
from CostMap.msg import cost_map_msgs
from geometry_msgs.msg import PoseStamped
from deneigus.msg import chute_msg, MBFcmd
from deneigus.srv import set_paths

from rospackage.src.config_utils import ConfigUtils


# Behavior for auto mode values
behavior_choices = {
    "semi_auto": 0,
    "nord": 1,
    "est": 2,
    "sud": 3,
    "west": 4,
    "centre": 5}


class global_target:
    def __init__(self, pose, soufflante, chute):
        self._pose = pose
        self._soufflante = soufflante
        self._chute = chute

    @property
    def pose(self):
        return self._pose

    @property
    def soufflante(self):
        return self._soufflante

    @property
    def chute(self):
        return self._chute


class GlobalPlan:
    def __init__(self):
        self.logger = get_logger("global_planner.main")
        self.logger.debug("Started global_planner init")

        self.turn_radius = 1.5   # turn radius of the robot (m)
        self.slice_width = 0.4   # distance between passe (m)
        self.map_grid = []
        self.map_resolution = 0
        self.v_ext_forward = [1, 0.3]
        self.v_ext_backward = [-1, -0.3]
        self.v_ext_none = [0, 0]
        self.empty_mbf_pose = self.createPose(None, None, None, None, None, None, None, self.v_ext_none[0], self.v_ext_none[1])

        # Publisher for robot's control TODO: change to service set_paths
        self.global_target = rospy.Publisher('/global_target', Float32MultiArray, queue_size=10)

        # load config -> start, side_street, snow_out_area
        config_utils = ConfigUtils()
        self.map_config = config_utils.load_yaml(origine_path=os.getcwd() + '/../catkin_ws/src/deneigus/map/config.yaml')
        # TODO : get params to wind (vector) and snow density


        # Subscriber from nodes or robot
        rospy.Subscriber('behavior', Int32, self.auto_plan_callback)

        # TODO : validate subscribe to global map
        rospy.Subscriber('/move_base_flex/global_costmap/costmap', cost_map_msgs, self.map_callback)

        self.logger.debug("Finished global_planner init")

    def createPose(self, x, y, z, xx, yy, zz, ww, vmax, vmin):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = rospy.get_rostime()
        p.pose.position.x = x
        p.pose.position.y = y
        p.pose.position.z = z
        p.pose.orientation.x = xx
        p.pose.orientation.y = yy
        p.pose.orientation.z = zz
        p.pose.orientation.w = ww

        a = MBFcmd()
        a.pose = p
        a.v_max = vmax
        a.v_min = vmin
        return a

    def createChute(self, x, y, force45):
        c = chute_msg()
        c.x = x
        c.y = y
        c.force45 = force45
        return c

    def map_callback(self, msg):
        self.grid_map = msg.data.data
        self.map_resolution = msg.info.resolution

        self.logger.debug("Costmap recieved")

    def auto_plan_callback(self, msg):
        # Constant values to simplify
        start_pos = self.map_config['start']

        width_driveway = 0
        index_pos_live = round(start_pos / self.map_resolution)
        while index_pos_live[1] != 0:  # TODO: validate 0 values in costmap
            width_driveway += self.map_resolution
            index_pos_live[1] += 1

        len_driveway = 0
        index_pos_live = round(start_pos / self.map_resolution)
        while index_pos_live[0] != 0:
            len_driveway += self.map_resolution
            index_pos_live[0] += 1

        # Planning
        if msg.data == behavior_choices.get("semi_auto"):
            self.logger.warning(f"Behavior {msg.data}, not implemented yet")

        elif msg.data == behavior_choices.get("nord"):
            self.logger.warning(f"Behavior {msg.data}, not implemented yet")

        elif msg.data == behavior_choices.get("est"):
            path_mbf, path_soufflante, path_chute = self.plan_est(start_pos, width_driveway, len_driveway)
            self.logger.warning(f"Behavior {msg.data}, in progress")

        elif msg.data == behavior_choices.get("sud"):
            self.logger.warning(f"Behavior {msg.data}, not implemented yet")

        elif msg.data == behavior_choices.get("west"):
            self.logger.warning(f"Behavior {msg.data}, not implemented yet")

        elif msg.data == behavior_choices.get("centre"):
            self.logger.warning(f"Behavior {msg.data}, not implemented yet")

        else:
            self.logger.fatal(f"Behavior {msg.data} not supported")

        rospy.wait_for_service('set_paths')
        set_func = rospy.ServiceProxy('set_paths', set_paths)
        success = set_func(path_mbf, path_soufflante,  path_chute)

        self.logger.info(success)


    def plan_est(self, sp, width_driveway, len_driveway):
        list_pos_target = []
        list_soufflante_target = []
        list_chute_target = []

        # Dont move and stop chute
        list_pos_target.append(self.empty_mbf_pose)
        list_soufflante_target.append(0)  # down=-1, stop=0, up=1
        list_chute_target.append(self.createChute(0, 0, False))
        # Dont move and up soufflante
        list_pos_target.append(self.empty_mbf_pose)
        list_soufflante_target.append(1)  # down=-1, stop=0, up=1
        list_chute_target.append(self.createChute(0, 0, False))
        # Go to start point
        list_pos_target.append(self.createPose(sp[0], sp[1], sp[2], sp[3], sp[4], sp[5], sp[6], self.v_ext_forward[0], self.v_ext_forward[1]))
        list_soufflante_target.append(0)  # down=-1, stop=0, up=1
        list_chute_target.append(self.createChute(0, 0, False))

        # Down soufflante
        list_pos_target.append(self.createPose(self.empty_mbf_pose))
        list_soufflante_target.append(-1)  # down=-1, stop=0, up=1
        list_chute_target.append(self.createChute(0, 0, False))
        # Start blow left
        list_pos_target.append(self.createPose(self.empty_mbf_pose))
        list_soufflante_target.append(0)  # down=-1, stop=0, up=1
        list_chute_target.append(self.createChute(0, 2, False))
        # Go to first point while blowing
        q = tf.transformations.quaternion_from_euler(0, 0, 0)  # roll, pitch, yaw
        list_pos_target.append(self.createPose(len_driveway-self.turn_radius, sp[1], 0, q[0], q[1], q[2], q[3], self.v_ext_forward[0], self.v_ext_forward[1]))
        list_soufflante_target.append(0)  # down=-1, stop=0, up=1
        list_chute_target.append(self.createChute(0, 2, False))

        # Stop blow
        list_pos_target.append(self.createPose(self.empty_mbf_pose))
        list_soufflante_target.append(0)  # down=-1, stop=0, up=1
        list_chute_target.append(self.createChute(0, 0, False))
        # Up soufflante
        list_pos_target.append(self.createPose(self.empty_mbf_pose))
        list_soufflante_target.append(1)  # down=-1, stop=0, up=1
        list_chute_target.append(self.createChute(0, 0, False))
        # Turn 90deg and slide
        q = tf.transformations.quaternion_from_euler(0, 0, 180)  # roll, pitch, yaw
        list_pos_target.append(self.createPose(len_driveway - self.turn_radius, sp[1]+self.slice_width, 0, q[0], q[1], q[2], q[3], self.v_ext_forward[0], self.v_ext_forward[1]))
        list_soufflante_target.append(0)  # down=-1, stop=0, up=1
        list_chute_target.append(self.createChute(0, 0, False))

        # Down soufflante
        list_pos_target.append(self.createPose(self.empty_mbf_pose))
        list_soufflante_target.append(-1)  # down=-1, stop=0, up=1
        list_chute_target.append(self.createChute(0, 0, False))
        # Start blow right
        list_pos_target.append(self.createPose(self.empty_mbf_pose))
        list_soufflante_target.append(0)  # down=-1, stop=0, up=1
        list_chute_target.append(self.createChute(0, -2, False))
        # Go to second point while blowing
        list_pos_target.append(self.createPose(sp[0], sp[1]+self.slice_width, 0, q[0], q[1], q[2], q[3], self.v_ext_forward[0], self.v_ext_forward[1]))
        list_soufflante_target.append(0)  # down=-1, stop=0, up=1
        list_chute_target.append(self.createChute(0, -2, False))

        # Rest of the driveway
        # nbr_slice = math.ceil(len_driveway/self.slice_width)
        # for i in range(nbr_slice):
        #     list_pos_target.append()
        #     list_soufflante_target.append()
        #     list_chute_target.append()

        return list_pos_target, list_soufflante_target, list_chute_target

    def goto_no_blow(self):

        return False

    def goto_blow(self):

        return False


if __name__ == "__main__":
    rospy.init_node('global_planner', anonymous=False)

    setup_logger(__file__, print_level=logging.INFO)
    logger = get_logger("global_planner")
    logger.info("Global_planner main Started")


    GlobalPlan()
    rospy.spin()


    logger.info("Global_planner main Stopped")