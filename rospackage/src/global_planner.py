
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


'''
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
'''


class GlobalPlan:
    def __init__(self):
        self.logger = get_logger("global_planner.main")
        self.logger.debug("Started global_planner init")

        self.turn_radius = 1.5   # turn radius of the robot (m)
        self.slice_width = 0.4   # distance between passe (m)
        self.map_grid = []
        self.map_resolution = 0
        self.max_costmap = 100
        # v_ext not used anymore
        self.v_ext_forward = [1, 0.3]
        self.v_ext_backward = [-1, -0.3]
        self.v_ext_none = [0, 0]
        self.empty_mbf_pose = self.create_pose(None, None, None, None, None, None, None)

        # load config -> start, side_street, snow_out_area
        config_utils = ConfigUtils()
        self.map_config = config_utils.load_yaml(origine_path=os.getcwd() + '/../catkin_ws/src/deneigus/map/config.yaml')
        # TODO : get params to wind (vector) and snow density

        # Subscriber from nodes or robot
        rospy.Subscriber('behavior', Int32, self.auto_plan_callback)

        # TODO : validate subscribe to global map
        rospy.Subscriber('/move_base_flex/global_costmap/costmap', cost_map_msgs, self.map_callback)

        self.logger.debug("Finished global_planner init")

    def create_pose(self, x, y, z, xx, yy, zz, ww):
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
        a.v_max = 0
        a.v_min = 0
        return a

    def create_chute(self, x, y, force45):
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
        while index_pos_live[1] != self.max_costmap:  # TODO: validate 100 values in costmap -> done -> test in simulation
            width_driveway += self.map_resolution
            index_pos_live[1] += 1

        len_driveway = 0
        index_pos_live = round(start_pos / self.map_resolution)
        while index_pos_live[0] != self.max_costmap:
            len_driveway += self.map_resolution
            index_pos_live[0] += 1

        max_dist_x = len_driveway - self.turn_radius
        max_dist_y = width_driveway - self.turn_radius

        nbr_slice_x = math.ceil(len_driveway / self.slice_width)
        nbr_slice_y = math.ceil(width_driveway / self.slice_width)

        # Planning
        if msg.data == behavior_choices.get("semi_auto"):
            self.logger.warning(f"Behavior {msg.data}, not implemented yet")

        elif msg.data == behavior_choices.get("nord"):
            self.logger.warning(f"Behavior {msg.data}, not implemented yet")

        elif msg.data == behavior_choices.get("est"):
            path_mbf, path_soufflante, path_chute = self.plan_2slice_back_and_fort(start_pos, max_dist_x, max_dist_y, nbr_slice_x)
            self.logger.warning(f"Behavior {msg.data}, in progress")

        elif msg.data == behavior_choices.get("sud"):
            self.logger.warning(f"Behavior {msg.data}, not implemented yet")

        elif msg.data == behavior_choices.get("west"):
            self.logger.warning(f"Behavior {msg.data}, not implemented yet")

        elif msg.data == behavior_choices.get("centre"):
            self.logger.warning(f"Behavior {msg.data}, not implemented yet")

        else:
            self.logger.fatal(f"Behavior {msg.data} not supported")

        # Send to state manager
        try:
            rospy.wait_for_service('set_paths', timeout=10)     # 10 sec timeout
        except rospy.ROSException:
            self.logger.fatal(f'Set_path service not reached')
        set_func = rospy.ServiceProxy('set_paths', set_paths)
        success = set_func(path_mbf, path_soufflante,  path_chute)
        self.logger.info(success)

    def plan_2slice_back_and_fort(self, sp, max_dist_x, max_dist_y, nbr_slice):
        pos_targets = []
        soufflante_targets = []
        chute_targets = []

        start_x, start_y, start_angle = sp

        # Start point
        pos_targets, soufflante_targets, chute_targets = self.goto_no_blow(pos_targets, soufflante_targets, chute_targets, start_x, start_y, start_angle)

        # First slice
        pos_targets, soufflante_targets, chute_targets = self.goto_blow(pos_targets, soufflante_targets, chute_targets, max_dist_x, start_y, start_angle, 0, 5, False)

        # rotate 180deg and slide
        pos_targets, soufflante_targets, chute_targets = self.slide_zigzag(pos_targets, soufflante_targets, chute_targets, max_dist_x, start_y+self.slice_width, start_angle+180)

        # Second slice
        pos_targets, soufflante_targets, chute_targets = self.goto_blow(pos_targets, soufflante_targets, chute_targets, start_x, start_y+self.slice_width, start_angle+180, 0, -5, False)

        # Prep for rest of the driveway
        pos_targets, soufflante_targets, chute_targets = self.goto_no_blow(pos_targets, soufflante_targets, chute_targets, start_x, start_y+self.slice_width, start_angle+90)

        # Rest of the driveway
        for i in range(nbr_slice):
            pos_targets, soufflante_targets, chute_targets = self.goto_blow(pos_targets, soufflante_targets, chute_targets, start_x+(i*self.slice_width), max_dist_y, start_angle+90, 6, 0, True)
            if i < nbr_slice:
                pos_targets, soufflante_targets, chute_targets = self.slide_backnext(pos_targets, soufflante_targets, chute_targets, start_x+((i+1)*self.slice_width), start_y+self.slice_width, start_angle+90)

        return pos_targets, soufflante_targets, chute_targets

    # niv 1
    def append_path(self, Lpose, Lsouffl, Lchute, pose_x, pose_y, pose_rot, souffl_h, chute_x, chute_y, chute_45):
        q = tf.transformations.quaternion_from_euler(0, 0, pose_rot)  # roll, pitch, yaw
        Lpose.append(self.create_pose(pose_x, pose_y, 0, q[0], q[1], q[2], q[3]))
        Lsouffl.append(souffl_h)  # down=-1, stop=0, up=1
        Lchute.append(self.create_chute(chute_x, chute_y, chute_45))
        return Lpose, Lsouffl, Lchute

    # niv 2
    def goto_no_blow(self, Lpose, Lsouffl, Lchute, pose_x, pose_y, pose_rot):
        Lpose, Lsouffl, Lchute = self.append_path(Lpose, Lsouffl, Lchute, None, None, None, 0, 0, 0, False)  # Dont move and stop chute
        Lpose, Lsouffl, Lchute = self.append_path(Lpose, Lsouffl, Lchute, None, None, None, 1, 0, 0, False)  # Dont move and up soufflante
        Lpose, Lsouffl, Lchute = self.append_path(Lpose, Lsouffl, Lchute, pose_x, pose_y, pose_rot, 0, 0, 0, False)  # Go to point
        return Lpose, Lsouffl, Lchute

    # niv 2
    def goto_blow(self, Lpose, Lsouffl, Lchute, pose_x, pose_y, pose_rot, blow_x, blow_y, blowout):
        Lpose, Lsouffl, Lchute = self.append_path(Lpose, Lsouffl, Lchute, None, None, None, -1, 0, 0, False)
        Lpose, Lsouffl, Lchute = self.append_path(Lpose, Lsouffl, Lchute, None, None, None, 0, blow_x, blow_y, blowout)
        Lpose, Lsouffl, Lchute = self.append_path(Lpose, Lsouffl, Lchute, pose_x, pose_y, pose_rot, 0, blow_x, blow_y, blowout)
        return Lpose, Lsouffl, Lchute

    # niv 3
    def slide_zigzag(self, Lpose, Lsouffl, Lchute, pose_x, pose_y, pose_rot):
        Lpose, Lsouffl, Lchute = self.goto_no_blow(Lpose, Lsouffl, Lchute, pose_x, pose_y-self.slice_width, pose_rot-90)
        Lpose, Lsouffl, Lchute = self.goto_blow(Lpose, Lsouffl, Lchute, pose_x, pose_y, pose_rot-90)
        Lpose, Lsouffl, Lchute = self.goto_no_blow(Lpose, Lsouffl, Lchute, pose_x, pose_y, pose_rot)
        return Lpose, Lsouffl, Lchute

    # niv 3
    def slide_backnext(self, Lpose, Lsouffl, Lchute, pose_x, pose_y, pose_rot):
        Lpose, Lsouffl, Lchute = self.goto_no_blow(Lpose, Lsouffl, Lchute, pose_x-self.slice_width, pose_y-self.slice_width, pose_rot-90)
        Lpose, Lsouffl, Lchute = self.goto_no_blow(Lpose, Lsouffl, Lchute, pose_x, pose_y, pose_rot - 90)
        return Lpose, Lsouffl, Lchute


if __name__ == "__main__":
    rospy.init_node('global_planner', anonymous=False)

    setup_logger(__file__, print_level=logging.INFO)
    logger = get_logger("global_planner")
    logger.info("Global_planner main Started")

    GlobalPlan()
    rospy.spin()

    logger.info("Global_planner main Stopped")