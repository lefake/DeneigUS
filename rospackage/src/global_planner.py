#!/usr/bin/env python3
import logging
import math

from logging_utils import setup_logger, get_logger
import rospy
import os
import tf
import numpy as np
from numpy import deg2rad as d2r

from std_msgs.msg import Float32MultiArray, Int32
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
from deneigus.msg import chute_msg, mbf_msg
from deneigus.srv import set_paths
from nav_msgs.srv import GetMap

from common_utils import ConfigYaml, behavior_choices


class GlobalPlan:
    def __init__(self):
        self.logger = get_logger("global_planner.main")
        self.logger.debug("Started global_planner init")

        self.turn_radius = 1.0   # turn radius of the robot (m)
        self.slice_width = 0.6   # distance between passe (m) -> environ 20po
        self.snow_throw_dist = 6 # m
        # v_ext for future use
        self.v_ext_forward = [1, 0.3]
        self.v_ext_backward = [-1, -0.3]
        self.v_ext_none = [0, 0]

        # load config -> start, side_street, snow_out_area
        config_utils = ConfigYaml()
        self.map_config = config_utils.load_yaml(path=os.getcwd() + '/../catkin_ws/src/deneigus/map/global_planner_params.yaml')
        # TODO : get params to wind (vector) and snow density

        rospy.wait_for_service('static_map')
        static_map_func = rospy.ServiceProxy('static_map', GetMap)
        static_map = static_map_func().map
        self.map_grid = np.reshape(static_map.data, (static_map.info.height, static_map.info.width))
        self.map_resolution = static_map.info.resolution
        self.map_obstacle_value = 100

        # Subscriber from nodes or robot
        rospy.Subscriber('behavior', Int32, self.auto_plan_callback)

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

        a = mbf_msg()
        a.pose = p
        a.v_max = 0.0
        a.v_min = 0.0
        return a

    def create_chute(self, x, y, force45):
        c = chute_msg()
        c.x = x
        c.y = y
        c.force45 = force45
        return c

    def auto_plan_callback(self, msg):
        # Planning
        behavior_name_list = list(behavior_choices.keys())
        if msg.data == behavior_choices.get("BF"):
            path_mbf, path_soufflante, path_chute = self.plan_BF(self.map_config['start'], mirror=False)
            self.logger.info(f"Behavior {behavior_name_list[msg.data]}")

        elif msg.data == behavior_choices.get("BFM"):
            path_mbf, path_soufflante, path_chute = self.plan_BF(self.map_config['startM'], mirror=True)
            self.logger.info(f"Behavior {behavior_name_list[msg.data]}")

        elif msg.data == behavior_choices.get("RE"):
            path_mbf, path_soufflante, path_chute = self.plan_RE(self.map_config['start'], mirror=False)
            self.logger.info(f"Behavior {behavior_name_list[msg.data]}")

        elif msg.data == behavior_choices.get("REM"):
            path_mbf, path_soufflante, path_chute = self.plan_RE(self.map_config['startM'], mirror=True)
            self.logger.info(f"Behavior {behavior_name_list[msg.data]}")

        elif msg.data == behavior_choices.get("ZZ"):
            path_mbf, path_soufflante, path_chute = self.plan_ZZ(self.map_config['start'], mirror=False)
            self.logger.info(f"Behavior {behavior_name_list[msg.data]}")

        elif msg.data == behavior_choices.get("ZZM"):
            path_mbf, path_soufflante, path_chute = self.plan_ZZ(self.map_config['startM'], mirror=True)
            self.logger.info(f"Behavior {behavior_name_list[msg.data]}")

        else:
            self.logger.fatal(f"Behavior {msg.data} not supported")

        # Send to state manager
        try:
            rospy.wait_for_service('set_paths', timeout=10)     # 10 sec timeout
            set_func = rospy.ServiceProxy('set_paths', set_paths)
            success = set_func(path_mbf, path_soufflante, path_chute)
        except rospy.ROSException:
            self.logger.fatal(f'Set_path service not reached')
            success = False
        self.logger.info(success)

    def map_analysis(self, start_x, start_y, mirror):
        # The planner is robust to rotations according to the starting point
        # The mirror parameter allows to cover all the other use cases
        M = -1 if mirror else 1

        len_driveway = 0
        index_pos_start_x = round(start_x / self.map_resolution)
        index_pos_start_y = round(start_y / self.map_resolution)
        while self.map_grid[index_pos_start_y, index_pos_start_x] < self.map_obstacle_value:
            len_driveway += self.map_resolution
            index_pos_start_x += 1

        width_driveway = 0
        index_pos_start_x = round(start_x / self.map_resolution)
        index_pos_start_y = round(start_y / self.map_resolution)
        while self.map_grid[index_pos_start_y, index_pos_start_x] < self.map_obstacle_value:
            width_driveway = width_driveway + (self.map_resolution * M)
            index_pos_start_y = index_pos_start_y + M

        nbr_slice_x = math.ceil(len_driveway / self.slice_width)
        nbr_slice_y = math.ceil(width_driveway / self.slice_width)
        # TODO: snowblower doesnt handle half-slice

        max_dist_x = start_x + len_driveway - self.turn_radius
        max_dist_y = start_y + width_driveway - (self.turn_radius * M)

        self.logger.debug(f'len_driveway: {len_driveway}')
        self.logger.debug(f'width_driveway: {width_driveway}')
        self.logger.debug(f'max_dist_x: {max_dist_x}')
        self.logger.debug(f'max_dist_y: {max_dist_y}')

        return nbr_slice_x, nbr_slice_y, max_dist_x, max_dist_y, M

    def plan_BF(self, sp, mirror):
        p_targets = []
        s_targets = []
        c_targets = []

        # Constant values to simplify
        start_x, start_y, start_angle = sp
        start_angle = d2r(start_angle)

        nbr_slice_x, nbr_slice_y, max_dist_x, max_dist_y, M = self.map_analysis(start_x, start_y, mirror)

        # PLANIFICATION Start point
        p_targets, s_targets, c_targets = self.goto_no_blow(p_targets, s_targets, c_targets, start_x, start_y, start_angle)
        # First slice
        p_targets, s_targets, c_targets = self.goto_blow(p_targets, s_targets, c_targets, max_dist_x, start_y, start_angle, 0, self.snow_throw_dist*M, False)
        # rotate 180deg and slide
        p_targets, s_targets, c_targets = self.slide_zigzag(p_targets, s_targets, c_targets, max_dist_x, start_y+(self.slice_width*M), start_angle+(d2r(180)*M), M, M)
        # Second slice
        p_targets, s_targets, c_targets = self.goto_blow(p_targets, s_targets, c_targets, start_x, start_y+(self.slice_width*M), start_angle+(d2r(180)*M), 0, -self.snow_throw_dist*M, False)
        # Prep for rest of the driveway
        p_targets, s_targets, c_targets = self.goto_no_blow(p_targets, s_targets, c_targets, start_x, start_y+(self.slice_width*M), start_angle+(d2r(90)*M))

        # Rest of the driveway
        for i in range(nbr_slice_x):
            p_targets, s_targets, c_targets = self.goto_blow(p_targets, s_targets, c_targets, start_x+(i*self.slice_width), max_dist_y, start_angle+(d2r(90)*M), self.snow_throw_dist, 0, True)

            if i == nbr_slice_x-1:
                p_targets, s_targets, c_targets = self.goto_no_blow(p_targets, s_targets, c_targets, start_x,  start_y, start_angle)

            elif i < nbr_slice_x:
                p_targets, s_targets, c_targets = self.slide_backnext(p_targets, s_targets, c_targets, start_x+((i+1)*self.slice_width), start_y+(self.slice_width*M), start_angle+(d2r(90)*M), M)


        return p_targets, s_targets, c_targets

    def plan_RE(self, sp, mirror):
        p_targets = []
        s_targets = []
        c_targets = []

        # Constant values to simplify
        start_x, start_y, start_angle = sp
        start_angle = d2r(start_angle)

        _, _, max_dist_x, max_dist_y, M = self.map_analysis(start_x, start_y, mirror)
        nbr_slice_r = round(self.turn_radius / self.slice_width)

        # PLANIFICATION Start point
        p_targets, s_targets, c_targets = self.goto_no_blow(p_targets, s_targets, c_targets, max_dist_x, start_y, start_angle)
        # First slice
        p_targets, s_targets, c_targets = self.goto_blow(p_targets, s_targets, c_targets, max_dist_x+self.turn_radius, start_y, start_angle, 0, self.snow_throw_dist*M, False)
        # rotate 180deg and slide
        p_targets, s_targets, c_targets = self.slide_zigzag(p_targets, s_targets, c_targets, max_dist_x+self.turn_radius, start_y + (self.slice_width*M), start_angle + (d2r(180)*M), M, M)
        # Second slice
        p_targets, s_targets, c_targets = self.goto_blow(p_targets, s_targets, c_targets, max_dist_x, start_y + (self.slice_width*M), start_angle + (d2r(180)*M), 0, -self.snow_throw_dist*M, False)
        # Prep for rest of the driveway
        p_targets, s_targets, c_targets = self.goto_no_blow(p_targets, s_targets, c_targets, max_dist_x, start_y + (self.slice_width*M), start_angle + (d2r(90)*M))

        # Rest of the driveway
        for i in range(nbr_slice_r):
            p_targets, s_targets, c_targets = self.goto_blow(p_targets, s_targets, c_targets, max_dist_x+(i*self.slice_width), max_dist_y, start_angle + (d2r(90)*M), self.snow_throw_dist, 0, True)

            if i < nbr_slice_r:
                p_targets, s_targets, c_targets = self.slide_backnext(p_targets, s_targets, c_targets, max_dist_x+((i+1)*self.slice_width), start_y + (self.slice_width*M), start_angle + (d2r(90)*M), M)

        return p_targets, s_targets, c_targets

    def plan_ZZ(self, sp, mirror):
        p_targets = []
        s_targets = []
        c_targets = []

        # Constant values to simplify
        start_x, start_y, start_angle = sp
        start_angle = d2r(start_angle)

        nbr_slice_x, nbr_slice_y, max_dist_x, max_dist_y, M = self.map_analysis(start_x, start_y, mirror)

        # First pass and turn
        p_targets, s_targets, c_targets = self.goto_no_blow(p_targets, s_targets, c_targets, start_x,  start_y, start_angle)
        p_targets, s_targets, c_targets = self.goto_blow(p_targets, s_targets, c_targets, max_dist_x, start_y, start_angle, 0, self.snow_throw_dist*M, False)

        # Rest of the driveway
        for i in range(nbr_slice_y):
            if i == nbr_slice_y-1:
                p_targets, s_targets, c_targets = self.goto_no_blow(p_targets, s_targets, c_targets, start_x,  start_y, start_angle)
            elif i%2 == 0:  # even number
                p_targets, s_targets, c_targets = self.slide_zigzag(p_targets, s_targets, c_targets, max_dist_x, start_y + ((i+1)*self.slice_width*M), start_angle + (d2r(180)*M), M, M)
                p_targets, s_targets, c_targets = self.goto_blow(p_targets, s_targets, c_targets, start_x, start_y + ((i+1)*self.slice_width*M), start_angle + (d2r(180)*M), 0, -self.snow_throw_dist*M, False)
            else:
                p_targets, s_targets, c_targets = self.slide_zigzag(p_targets, s_targets, c_targets, start_x, start_y + ((i+1)*self.slice_width*M), start_angle, M, -M)
                p_targets, s_targets, c_targets = self.goto_blow(p_targets, s_targets, c_targets, max_dist_x, start_y + ((i+1)*self.slice_width*M), start_angle, 0, self.snow_throw_dist*M, False)

        return p_targets, s_targets, c_targets

    # niv 1
    def append_path(self, Lpose, Lsouffl, Lchute, pose_x, pose_y, pose_rot, souffl_h, chute_x, chute_y, chute_45):
        if pose_rot is not None:
            q = tf.transformations.quaternion_from_euler(0, 0, pose_rot)  # roll, pitch, yaw
        else:
            q = [0.0, 0.0, 0.0, 0.0]
            pose_x = 0.0
            pose_y = 0.0

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
    def slide_zigzag(self, Lpose, Lsouffl, Lchute, pose_x, pose_y, pose_rot, M, F):
        Lpose, Lsouffl, Lchute = self.goto_no_blow(Lpose, Lsouffl, Lchute, pose_x, pose_y-(self.slice_width*M), pose_rot-(d2r(90)*F))
        Lpose, Lsouffl, Lchute = self.goto_blow(Lpose, Lsouffl, Lchute, pose_x, pose_y, pose_rot-(d2r(90)*F), 4, 0, False)
        Lpose, Lsouffl, Lchute = self.goto_no_blow(Lpose, Lsouffl, Lchute, pose_x, pose_y, pose_rot)
        return Lpose, Lsouffl, Lchute

    # niv 3
    def slide_backnext(self, Lpose, Lsouffl, Lchute, pose_x, pose_y, pose_rot, M):
        Lpose, Lsouffl, Lchute = self.goto_no_blow(Lpose, Lsouffl, Lchute, pose_x-self.slice_width, pose_y-(self.slice_width*M), pose_rot)
        Lpose, Lsouffl, Lchute = self.goto_blow(Lpose, Lsouffl, Lchute, pose_x, pose_y, pose_rot, 2, 0, True)
        return Lpose, Lsouffl, Lchute


if __name__ == "__main__":
    rospy.init_node('global_planner', anonymous=False)

    setup_logger(__file__, print_level=logging.INFO)
    logger = get_logger("global_planner")
    logger.info("Global_planner main Started")

    GlobalPlan()
    rospy.spin()

    logger.info("Global_planner main Stopped")