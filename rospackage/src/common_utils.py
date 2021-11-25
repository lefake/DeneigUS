
from logging_utils import get_logger
import yaml
from enum import Enum


class ConfigYaml:
    def __init__(self):
        self.logger = get_logger("config_utils.main")

    def load_yaml(self, path):
        with open(path, "r") as stream:
            try:
                data = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                self.logger.info(exc)
                data = None
        return data

    # TODO: write_yaml
    #def write_yaml(self, path):


# Control mode values
class control_modes(Enum):
    stop = 0
    manual = 1
    auto = 2


# Behavior for auto mode values
behavior_choices = {
    "BF": 0,
    "BFM": 1,
    "RE": 2,
    "REM": 3,
    "ZZ": 4,
    "ZZM": 5}


supported_type = ["ps3", "ps4", "logi"]


def joy_button_mapper(joy_type):
    joy_indexes = {}

    if joy_type == "ps3" or joy_type == "ps4":
        # Axes
        joy_indexes["prop_lin"] = 4
        joy_indexes["prop_ang"] = 3
        joy_indexes["chute_rot"] = 0
        joy_indexes["chute_elev"] = 1
        joy_indexes["soufl_speed"] = 5

        # Buttons
        joy_indexes["soufl_up"] = 2
        joy_indexes["soufl_down"] = 0
        joy_indexes["deadman"] = 4
        joy_indexes["switch_mode"] = 3
        joy_indexes["behavior_p"] = 12
        joy_indexes["behavior_m"] = 11
        joy_indexes["light"] = 1
        joy_indexes["estop"] = 9
        joy_indexes["reset"] = 8

    elif joy_type == "logi":
        # Axes
        joy_indexes["prop_lin"] = 4
        joy_indexes["prop_ang"] = 3
        joy_indexes["chute_rot"] = 0
        joy_indexes["chute_elev"] = 1
        joy_indexes["soufl_speed"] = 5

        # Buttons
        joy_indexes["soufl_up"] = 3
        joy_indexes["soufl_down"] = 0
        joy_indexes["deadman"] = 4
        joy_indexes["switch_mode"] = 2
        joy_indexes["behavior_p"] = 10
        joy_indexes["behavior_m"] = 9
        joy_indexes["light"] = 1
        joy_indexes["estop"] = 7
        joy_indexes["reset"] = 6

    return joy_indexes


def lin_ang_2_tank(lin, ang):
    # https://home.kendra.com/mauser/Joystick.html
    v = (1 - abs(ang)) * (lin / 1) + lin
    w = (1 - abs(lin)) * (ang / 1) + ang

    right_vel = (v + w) / 2
    left_vel = (v - w) / 2

    return [left_vel, right_vel]